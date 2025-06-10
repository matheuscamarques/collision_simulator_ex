defmodule CollisionSimulator.Physics do
  @moduledoc """
  Módulo de computação física baseado em Nx.
  """
  import Nx.Defn

  @typep batch_states :: %{
           pos: Nx.Tensor.t(),
           vel: Nx.Tensor.t(),
           radius: Nx.Tensor.t(),
           mass: Nx.Tensor.t()
         }
  @typep world_bounds :: %{min_x: float(), max_x: float(), min_y: float(), max_y: float()}

  # ... handle_wall_collision, get_colliding_pairs etc. permanecem os mesmos ...

  @doc """
  Função Elixir para colisão de uma partícula individual com as paredes.
  Usada pelo processo Particle.
  """
  def handle_wall_collision(particle, world_bounds) do
    %{pos: [x, y], vel: [vx, vy], radius: r} = particle

    {min_x, max_x, min_y, max_y} =
      {world_bounds.min_x, world_bounds.max_x, world_bounds.min_y, world_bounds.max_y}

    {vx, x} =
      cond do
        x - r < min_x -> {-vx, min_x + r}
        x + r > max_x -> {-vx, max_x - r}
        true -> {vx, x}
      end

    {vy, y} =
      cond do
        y - r < min_y -> {-vy, min_y + r}
        y + r > max_y -> {-vy, max_y - r}
        true -> {vy, y}
      end

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
    candidate_pairs[Nx.squeeze(collision_mask)] |> Nx.reshape({:auto, 2})
  end

  @doc """
  NOVO: Calcula as respostas de colisão (novas velocidades) para os pares fornecidos.
  Retorna uma lista de mapas com os índices e as novas velocidades, para serem despachados.
  """
  def calculate_collision_responses(states, colliding_pairs) do
    {i_indices, j_indices, new_vel_i, new_vel_j} =
      do_calculate_collision_responses(states, colliding_pairs)

    # Converte os tensores de resultado em uma lista de mapas no lado do Elixir.
    i_list = Nx.to_list(i_indices)
    j_list = Nx.to_list(j_indices)
    new_vel_i_list = Nx.to_list(new_vel_i)
    new_vel_j_list = Nx.to_list(new_vel_j)

    Enum.zip([i_list, j_list, new_vel_i_list, new_vel_j_list])
    |> Enum.map(fn {idx_i, idx_j, vel_i, vel_j} ->
      %{index_a: idx_i, index_b: idx_j, new_vel_a: vel_i, new_vel_b: vel_j}
    end)
  end

  # Função `defn` privada que faz o cálculo pesado.
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

    # Lógica de resolução de velocidade (sem correção de posição, pois as partículas farão isso).
    normal = pos_j - pos_i
    distance = Nx.sqrt(Nx.sum(normal * normal, axes: [1], keep_axes: true))
    unit_normal = normal / (distance + 1.0e-6)

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

    {i_indices, j_indices, new_vel_i, new_vel_j}
  end
end
