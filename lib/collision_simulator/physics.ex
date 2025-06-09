defmodule CollisionSimulator.Physics do
  @moduledoc """
  Módulo de computação física baseado em Nx.

  Técnicas:
  - Cálculos vetorizados em GPU/CPU via EXLA
  - Colisões elásticas com conservação de momento
  - Operações em lote com tensores

  Funções-chave:
  - batch_step: Atualização de movimento + colisão com paredes
  - resolve_all_collisions: Resolução de colisões partícula-partícula
  """
  import Nx.Defn

  # --- Tipos ---
  @typep batch_states :: %{
           pos: Nx.Tensor.t(),
           vel: Nx.Tensor.t(),
           radius: Nx.Tensor.t(),
           mass: Nx.Tensor.t()
         }

  @typep world_bounds :: %{
           min_x: float(),
           max_x: float(),
           min_y: float(),
           max_y: float()
         }

  @doc """
  Executa um passo completo da física para um lote de partículas.
  Isso inclui atualizar a posição com base na velocidade e lidar com colisões nas paredes.
  """
  @spec batch_step(batch_states(), float(), world_bounds()) :: batch_states()
  defn batch_step(states, dt, world_bounds) do
    # Pattern matching explícito para maior clareza.
    %{pos: positions, vel: velocities, radius: radii} = states

    # 1. Integração de Euler: calcula as novas posições com base na velocidade.
    # new_position = old_position + velocity * delta_time
    new_positions = positions + velocities * dt

    # 2. Lida com colisões nas fronteiras do mundo.
    {final_positions, final_velocities} =
      handle_wall_collisions(new_positions, velocities, radii, world_bounds)

    # Retorna o novo estado com as posições e velocidades atualizadas.
    %{states | pos: final_positions, vel: final_velocities}
  end

  @doc """
  Lida com colisões nas fronteiras do mundo para um lote de partículas.
  Se uma partícula ultrapassa uma parede, sua posição é corrigida e sua velocidade
  no eixo correspondente é invertida.
  """
  @spec handle_wall_collisions(Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t(), world_bounds()) ::
          {Nx.Tensor.t(), Nx.Tensor.t()}
  defn handle_wall_collisions(positions, velocities, radii, world_bounds_map) do
    min_x = world_bounds_map.min_x
    max_x = world_bounds_map.max_x
    min_y = world_bounds_map.min_y
    max_y = world_bounds_map.max_y

    # Separa os tensores de posição e velocidade em componentes x e y.
    pos_x = Nx.slice_along_axis(positions, 0, 1, axis: 1) |> Nx.squeeze(axes: [1])
    pos_y = Nx.slice_along_axis(positions, 1, 1, axis: 1) |> Nx.squeeze(axes: [1])
    vel_x = Nx.slice_along_axis(velocities, 0, 1, axis: 1) |> Nx.squeeze(axes: [1])
    vel_y = Nx.slice_along_axis(velocities, 1, 1, axis: 1) |> Nx.squeeze(axes: [1])
    # Remove a dimensão extra do tensor de raios.
    radii_flat = Nx.squeeze(radii)

    # --- Verificação de Colisão para cada Parede ---

    # Colisão com a parede esquerda (x < min_x)
    hit_left = pos_x - radii_flat < min_x
    # Inverte a velocidade x se colidir.
    vel_x = Nx.select(hit_left, -vel_x, vel_x)
    # Corrige a posição para ficar na borda.
    pos_x = Nx.select(hit_left, min_x + radii_flat, pos_x)

    # Colisão com a parede direita (x > max_x)
    hit_right = pos_x + radii_flat > max_x
    vel_x = Nx.select(hit_right, -vel_x, vel_x)
    pos_x = Nx.select(hit_right, max_x - radii_flat, pos_x)

    # Colisão com a parede superior (y < min_y)
    hit_top = pos_y - radii_flat < min_y
    vel_y = Nx.select(hit_top, -vel_y, vel_y)
    pos_y = Nx.select(hit_top, min_y + radii_flat, pos_y)

    # Colisão com a parede inferior (y > max_y)
    hit_bottom = pos_y + radii_flat > max_y
    vel_y = Nx.select(hit_bottom, -vel_y, vel_y)
    pos_y = Nx.select(hit_bottom, max_y - radii_flat, pos_y)

    # Remonta os tensores de posição e velocidade.
    final_positions = Nx.stack([pos_x, pos_y], axis: 1)
    final_velocities = Nx.stack([vel_x, vel_y], axis: 1)

    {final_positions, final_velocities}
  end

  @doc """
  Detecção de colisão otimizada:
  - Distância quadrática vs soma de raios quadrática
  - Evita uso de raiz quadrada
  - Operação vetorizada em todos os pares candidatos
  """
  @spec get_colliding_pairs(batch_states(), Nx.Tensor.t()) :: Nx.Tensor.t()
  defn get_colliding_pairs(states, candidate_pairs) do
    # Pattern matching explícito para maior clareza.
    %{pos: positions, radius: radii} = states

    # Obtém os índices das partículas i e j de cada par.
    i_indices = Nx.slice_along_axis(candidate_pairs, 0, 1, axis: 1) |> Nx.squeeze(axes: [1])
    j_indices = Nx.slice_along_axis(candidate_pairs, 1, 1, axis: 1) |> Nx.squeeze(axes: [1])

    # Usa `Nx.take` para obter os dados de cada partícula nos pares.
    pos_i = Nx.take(positions, i_indices)
    pos_j = Nx.take(positions, j_indices)
    r_i = Nx.take(radii, i_indices)
    r_j = Nx.take(radii, j_indices)

    # Calcula a distância ao quadrado entre as partículas.
    # (Usar a distância ao quadrado evita uma chamada `sqrt` cara).
    diff = pos_i - pos_j
    dist_sq = Nx.sum(diff * diff, axes: [1]) |> Nx.reshape({:auto, 1})

    # Calcula a soma dos raios ao quadrado.
    radius_sum = r_i + r_j
    min_dist_sq = radius_sum * radius_sum

    # Uma colisão ocorre se a distância ao quadrado for menor que a soma dos raios ao quadrado.
    collision_mask = dist_sq < min_dist_sq

    # Usa a máscara booleana para filtrar e retornar apenas os pares que colidem.
    candidate_pairs[Nx.squeeze(collision_mask)]
    # Garante que o shape de saída seja [N, 2].
    |> Nx.reshape({:auto, 2})
  end

  @doc """
  Resolução de colisões em 2 passos:
  1. Correção de posição (resolve penetração)
  2. Cálculo de impulso (atualiza velocidades)

  Fórmulas:
  - Correção: pos = pos ± (sobreposição / 2) * normal
  - Impulso: Δv = [2 * m₂ * (vᵢ - vⱼ) • n] / (mᵢ + mⱼ) * n
  """
  @spec resolve_all_collisions(batch_states(), Nx.Tensor.t()) :: batch_states()
  defn resolve_all_collisions(states, colliding_pairs) do
    # Desestrutura os tensores de estado.
    positions = states.pos
    velocities = states.vel
    masses = states.mass
    radii = states.radius

    # Extrai os índices das partículas colidindo.
    i_indices =
      Nx.slice_along_axis(colliding_pairs, 0, 1, axis: 1)
      |> Nx.squeeze(axes: [1])

    j_indices =
      Nx.slice_along_axis(colliding_pairs, 1, 1, axis: 1)
      |> Nx.squeeze(axes: [1])

    # Obtém os dados das partículas envolvidas na colisão.
    pos_i = Nx.take(positions, i_indices)
    pos_j = Nx.take(positions, j_indices)
    vel_i = Nx.take(velocities, i_indices)
    vel_j = Nx.take(velocities, j_indices)
    m_i = Nx.take(masses, i_indices)
    m_j = Nx.take(masses, j_indices)
    r_i = Nx.take(radii, i_indices)
    r_j = Nx.take(radii, j_indices)

    # --- Resolução de Posição e Velocidade ---

    # Calcula o vetor normal da colisão (de i para j).
    normal = pos_j - pos_i
    distance = Nx.sqrt(Nx.sum(normal * normal, axes: [1], keep_axes: true))

    # Adiciona um valor pequeno (epsilon) para evitar divisão por zero se a distância for 0.
    unit_normal = normal / (distance + 1.0e-6)

    # 1. Correção de Posição (para resolver a interpenetração)
    overlap = r_i + r_j - distance
    # Apenas corrige se houver uma sobreposição real, empurrando as partículas para fora.
    correction = Nx.select(overlap > 1.0e-6, overlap * 0.5, 0.0)
    new_pos_i = pos_i - unit_normal * correction
    new_pos_j = pos_j + unit_normal * correction

    # 2. Resolução de Velocidade (cálculo de impulso elástico)
    velocity_diff = vel_i - vel_j
    dot_product = Nx.sum(velocity_diff * unit_normal, axes: [1], keep_axes: true)

    # O impulso de colisão só deve ser aplicado se as partículas estiverem se movendo
    # uma em direção à outra (produto escalar > 0).
    is_approaching = dot_product > 0

    total_mass = m_i + m_j
    # A fórmula para o impulso em uma colisão elástica.
    impulse = 2.0 * dot_product / total_mass * unit_normal

    # Aplica o impulso apenas se as partículas estiverem se aproximando.
    # `is_approaching` é transmitida (broadcast) para corresponder à forma do impulso.
    broadcasted_mask = Nx.broadcast(is_approaching, Nx.shape(impulse))
    zero_impulse = Nx.broadcast(0.0, Nx.shape(impulse))
    effective_impulse = Nx.select(broadcasted_mask, impulse, zero_impulse)

    # As novas velocidades são calculadas com base no impulso efetivo e nas massas.
    new_vel_i = vel_i - effective_impulse * m_j
    new_vel_j = vel_j + effective_impulse * m_i

    # --- Atualização dos Tensores Globais ---
    # Usa `Nx.indexed_put` para atualizar os tensores globais de posição e velocidade
    # apenas para as partículas que colidiram.
    indices_i = Nx.reshape(i_indices, {:auto, 1})
    indices_j = Nx.reshape(j_indices, {:auto, 1})

    final_positions =
      positions
      |> Nx.indexed_put(indices_i, new_pos_i)
      |> Nx.indexed_put(indices_j, new_pos_j)

    final_velocities =
      velocities
      |> Nx.indexed_put(indices_i, new_vel_i)
      |> Nx.indexed_put(indices_j, new_vel_j)

    # Retorna o estado final atualizado.
    %{states | pos: final_positions, vel: final_velocities}
  end

  @doc """
  Uma função utilitária Elixir (não-Nx) para uma checagem precisa de colisão
  círculo-círculo. Não é usada no pipeline principal, mas pode ser útil para testes ou depuração.
  """
  @spec check_circle_collision(map(), map()) :: boolean()
  def check_circle_collision(p1, p2) do
    [p1x, p1y] = p1.pos
    [p2x, p2y] = p2.pos
    dx = p1x - p2x
    dy = p1y - p2y
    distance_sq = dx * dx + dy * dy
    radius_sum = p1.radius + p2.radius
    distance_sq <= radius_sum * radius_sum
  end
end
