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
    new_positions = positions + velocities * dt

    # 2. Lida com colisões nas fronteiras do mundo.
    {final_positions, final_velocities} =
      handle_wall_collisions(new_positions, velocities, radii, world_bounds)

    # Retorna o novo estado com as posições e velocidades atualizadas.
    %{states | pos: final_positions, vel: final_velocities}
  end

  @doc """
  Lida com colisões nas fronteiras do mundo para um lote de partículas.
  """
  @spec handle_wall_collisions(Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t(), world_bounds()) ::
          {Nx.Tensor.t(), Nx.Tensor.t()}
  defn handle_wall_collisions(positions, velocities, radii, world_bounds_map) do
    min_x = world_bounds_map.min_x
    max_x = world_bounds_map.max_x
    min_y = world_bounds_map.min_y
    max_y = world_bounds_map.max_y

    pos_x = Nx.slice_along_axis(positions, 0, 1, axis: 1) |> Nx.squeeze(axes: [1])
    pos_y = Nx.slice_along_axis(positions, 1, 1, axis: 1) |> Nx.squeeze(axes: [1])
    vel_x = Nx.slice_along_axis(velocities, 0, 1, axis: 1) |> Nx.squeeze(axes: [1])
    vel_y = Nx.slice_along_axis(velocities, 1, 1, axis: 1) |> Nx.squeeze(axes: [1])
    radii_flat = Nx.squeeze(radii)

    # Colisão com a parede esquerda
    hit_left = pos_x - radii_flat < min_x
    vel_x = Nx.select(hit_left, -vel_x, vel_x)
    pos_x = Nx.select(hit_left, min_x + radii_flat, pos_x)

    # Colisão com a parede direita
    hit_right = pos_x + radii_flat > max_x
    vel_x = Nx.select(hit_right, -vel_x, vel_x)
    pos_x = Nx.select(hit_right, max_x - radii_flat, pos_x)

    # Colisão com a parede superior
    hit_top = pos_y - radii_flat < min_y
    vel_y = Nx.select(hit_top, -vel_y, vel_y)
    pos_y = Nx.select(hit_top, min_y + radii_flat, pos_y)

    # Colisão com a parede inferior
    hit_bottom = pos_y + radii_flat > max_y
    vel_y = Nx.select(hit_bottom, -vel_y, vel_y)
    pos_y = Nx.select(hit_bottom, max_y - radii_flat, pos_y)

    # Remonta os tensores de posição e velocidade.
    final_positions = Nx.stack([pos_x, pos_y], axis: 1)
    final_velocities = Nx.stack([vel_x, vel_y], axis: 1)

    {final_positions, final_velocities}
  end

  @doc """
  Função Elixir para colisão de uma partícula individual com as paredes.
  Usada pelo processo Particle.
  """
  def handle_wall_collision(particle, world_bounds) do
    # CORREÇÃO: Padrão de correspondência agora usa listas para pos e vel.
    %{pos: [x, y], vel: [vx, vy], radius: r} = particle

    {min_x, max_x, min_y, max_y} =
      {world_bounds.min_x, world_bounds.max_x, world_bounds.min_y, world_bounds.max_y}

    # Colisão com paredes verticais
    {vx, x} =
      cond do
        x - r < min_x -> {-vx, min_x + r}
        x + r > max_x -> {-vx, max_x - r}
        true -> {vx, x}
      end

    # Colisão com paredes horizontais
    {vy, y} =
      cond do
        y - r < min_y -> {-vy, min_y + r}
        y + r > max_y -> {-vy, max_y - r}
        true -> {vy, y}
      end

    # CORREÇÃO: Retorna duas listas.
    {[x, y], [vx, vy]}
  end

  @doc """
  Detecção de colisão otimizada para pares de partículas.
  """
  @spec get_colliding_pairs(batch_states(), Nx.Tensor.t()) :: Nx.Tensor.t()
  defn get_colliding_pairs(states, candidate_pairs) do
    %{pos: positions, radius: radii} = states

    i_indices = Nx.slice_along_axis(candidate_pairs, 0, 1, axis: 1) |> Nx.squeeze(axes: [1])
    j_indices = Nx.slice_along_axis(candidate_pairs, 1, 1, axis: 1) |> Nx.squeeze(axes: [1])

    pos_i = Nx.take(positions, i_indices)
    pos_j = Nx.take(positions, j_indices)
    r_i = Nx.take(radii, i_indices)
    r_j = Nx.take(radii, j_indices)

    diff = pos_i - pos_j
    dist_sq = Nx.sum(diff * diff, axes: [1], keep_axes: true)

    radius_sum = r_i + r_j
    min_dist_sq = radius_sum * radius_sum

    collision_mask = dist_sq < min_dist_sq

    candidate_pairs[Nx.squeeze(collision_mask)]
    |> Nx.reshape({:auto, 2})
  end

  @doc """
  Resolução de colisões para todos os pares identificados.
  """
  @spec resolve_all_collisions(batch_states(), Nx.Tensor.t()) :: batch_states()
  defn resolve_all_collisions(states, colliding_pairs) do
    positions = states.pos
    velocities = states.vel
    masses = states.mass
    radii = states.radius

    i_indices =
      Nx.slice_along_axis(colliding_pairs, 0, 1, axis: 1)
      |> Nx.squeeze(axes: [1])

    j_indices =
      Nx.slice_along_axis(colliding_pairs, 1, 1, axis: 1)
      |> Nx.squeeze(axes: [1])

    pos_i = Nx.take(positions, i_indices)
    pos_j = Nx.take(positions, j_indices)
    vel_i = Nx.take(velocities, i_indices)
    vel_j = Nx.take(velocities, j_indices)
    m_i = Nx.take(masses, i_indices)
    m_j = Nx.take(masses, j_indices)
    r_i = Nx.take(radii, i_indices)
    r_j = Nx.take(radii, j_indices)

    normal = pos_j - pos_i
    distance = Nx.sqrt(Nx.sum(normal * normal, axes: [1], keep_axes: true))
    unit_normal = normal / (distance + 1.0e-6)

    # Correção de Posição
    overlap = r_i + r_j - distance
    correction = Nx.select(overlap > 1.0e-6, overlap * 0.5, 0.0)
    new_pos_i = pos_i - unit_normal * correction
    new_pos_j = pos_j + unit_normal * correction

    # Resolução de Velocidade
    velocity_diff = vel_i - vel_j
    dot_product = Nx.sum(velocity_diff * unit_normal, axes: [1], keep_axes: true)
    is_approaching = dot_product > 0
    total_mass = m_i + m_j
    impulse = 2.0 * dot_product / total_mass * unit_normal

    broadcasted_mask = Nx.broadcast(is_approaching, Nx.shape(impulse))
    zero_impulse = Nx.broadcast(0.0, Nx.shape(impulse))
    effective_impulse = Nx.select(broadcasted_mask, impulse, zero_impulse)

    new_vel_i = vel_i - effective_impulse * m_j
    new_vel_j = vel_j + effective_impulse * m_i

    # Atualização dos Tensores Globais
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

    %{states | pos: final_positions, vel: final_velocities}
  end

  @doc """
  Função utilitária Elixir para checagem de colisão círculo-círculo.
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
