defmodule CollisionSimulator.Physics do
  @moduledoc """
  Módulo de computação de física, otimizado com `Nx` (`Numerical Elixir`).

  Este módulo contém funções puras para realizar os cálculos pesados da simulação,
  como detecção de colisão entre pares e resolução de colisões elásticas. O uso de `Nx`
  permite que esses cálculos sejam executados em lote em tensores, o que é significativamente
  mais rápido do que iterar e calcular para cada par individualmente em Elixir puro.
  Muitas funções são definidas com `defn`, que as compila para execução em CPU/GPU.
  """
  import Nx.Defn

  # --- Tipos ---

  @typep batch_states :: %{
           pos: Nx.Tensor.t(),
           vel: Nx.Tensor.t(),
           radius: Nx.Tensor.t(),
           mass: Nx.Tensor.t()
         }
  @typep world_bounds :: %{min_x: float(), max_x: float(), min_y: float(), max_y: float()}

  # --- Funções ---

  @doc """
  Função Elixir (não `defn`) para calcular a colisão de uma ÚNICA partícula com
  as paredes do mundo. É usada por cada processo `Particle` individualmente.
  Retorna a nova posição e velocidade da partícula.
  """
  def handle_wall_collision(particle, world_bounds) do
    %{pos: [x, y], vel: [vx, vy], radius: r} = particle

    {min_x, max_x, min_y, max_y} =
      {world_bounds.min_x, world_bounds.max_x, world_bounds.min_y, world_bounds.max_y}

    # Verifica e resolve colisões no eixo X.
    {vx, x} =
      cond do
        # Colisão à esquerda
        x - r < min_x -> {-vx, min_x + r}
        # Colisão à direita
        x + r > max_x -> {-vx, max_x - r}
        true -> {vx, x}
      end

    # Verifica e resolve colisões no eixo Y.
    {vy, y} =
      cond do
        # Colisão superior
        y - r < min_y -> {-vy, min_y + r}
        # Colisão inferior
        y + r > max_y -> {-vy, max_y - r}
        true -> {vy, y}
      end

    {[x, y], [vx, vy]}
  end

  @doc """
  Função `defn` compilada para detecção de colisão otimizada para um lote de pares de partículas.
  Recebe os estados de todas as partículas e um tensor de pares candidatos.
  Retorna um novo tensor contendo apenas os pares que estão de fato colidindo.
  """
  @spec get_colliding_pairs(batch_states(), Nx.Tensor.t()) :: Nx.Tensor.t()
  defn get_colliding_pairs(states, candidate_pairs) do
    %{pos: positions, radius: radii} = states

    # Extrai os índices i e j dos pares candidatos.
    i_indices = Nx.slice_along_axis(candidate_pairs, 0, 1, axis: 1) |> Nx.squeeze(axes: [1])
    j_indices = Nx.slice_along_axis(candidate_pairs, 1, 1, axis: 1) |> Nx.squeeze(axes: [1])

    # Pega os dados das partículas correspondentes aos índices.
    pos_i = Nx.take(positions, i_indices)
    pos_j = Nx.take(positions, j_indices)
    r_i = Nx.take(radii, i_indices)
    r_j = Nx.take(radii, j_indices)

    # Lógica de detecção de colisão.
    diff = pos_i - pos_j
    # Distância ao quadrado (mais rápido)
    dist_sq = Nx.sum(diff * diff, axes: [1], keep_axes: true)
    radius_sum = r_i + r_j
    min_dist_sq = radius_sum * radius_sum

    # Cria uma máscara booleana para as colisões.
    collision_mask = dist_sq < min_dist_sq

    # Filtra os pares candidatos usando a máscara.
    candidate_pairs[Nx.squeeze(collision_mask)] |> Nx.reshape({:auto, 2})
  end

  @doc """
  Calcula as respostas de colisão (novas velocidades) para os pares que estão colidindo.
  Esta é uma função de invólucro (wrapper) em Elixir que chama a função `defn` interna
  e depois formata a saída de tensores para uma lista de mapas, que é mais fácil
  de ser manipulada pelo `CollisionEngine`.
  """
  def calculate_collision_responses(states, colliding_pairs) do
    {i_indices, j_indices, new_vel_i, new_vel_j} =
      do_calculate_collision_responses(states, colliding_pairs)

    # Converte os tensores de resultado de volta para listas Elixir.
    i_list = Nx.to_list(i_indices)
    j_list = Nx.to_list(j_indices)
    new_vel_i_list = Nx.to_list(new_vel_i)
    new_vel_j_list = Nx.to_list(new_vel_j)

    # Combina as listas em uma lista de mapas.
    Enum.zip([i_list, j_list, new_vel_i_list, new_vel_j_list])
    |> Enum.map(fn {idx_i, idx_j, vel_i, vel_j} ->
      %{index_a: idx_i, index_b: idx_j, new_vel_a: vel_i, new_vel_b: vel_j}
    end)
  end

  # Função `defn` privada que realiza o cálculo pesado da resposta à colisão.
  defn do_calculate_collision_responses(states, colliding_pairs) do
    # Extrai dados dos tensores globais usando os índices dos pares.
    i_indices = Nx.slice_along_axis(colliding_pairs, 0, 1, axis: 1) |> Nx.squeeze(axes: [1])
    j_indices = Nx.slice_along_axis(colliding_pairs, 1, 1, axis: 1) |> Nx.squeeze(axes: [1])

    pos_i = Nx.take(states.pos, i_indices)
    pos_j = Nx.take(states.pos, j_indices)
    vel_i = Nx.take(states.vel, i_indices)
    vel_j = Nx.take(states.vel, j_indices)
    m_i = Nx.take(states.mass, i_indices)
    m_j = Nx.take(states.mass, j_indices)

    # Lógica de resolução de colisão elástica 2D.
    normal = pos_j - pos_i
    distance = Nx.sqrt(Nx.sum(normal * normal, axes: [1], keep_axes: true))
    # Adiciona epsilon para evitar divisão por zero.
    unit_normal = normal / (distance + 1.0e-6)

    velocity_diff = vel_i - vel_j
    dot_product = Nx.sum(velocity_diff * unit_normal, axes: [1], keep_axes: true)

    # A colisão só é resolvida se as partículas estiverem se aproximando.
    is_approaching = dot_product > 0
    total_mass = m_i + m_j
    impulse = 2.0 * dot_product / total_mass * unit_normal

    # Aplica o impulso apenas se as partículas estiverem se aproximando.
    broadcasted_mask = Nx.broadcast(is_approaching, Nx.shape(impulse))
    zero_impulse = Nx.broadcast(0.0, Nx.shape(impulse))
    effective_impulse = Nx.select(broadcasted_mask, impulse, zero_impulse)

    # Calcula as novas velocidades.
    new_vel_i = vel_i - effective_impulse * m_j
    new_vel_j = vel_j + effective_impulse * m_i

    {i_indices, j_indices, new_vel_i, new_vel_j}
  end
end
