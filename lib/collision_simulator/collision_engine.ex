defmodule CollisionSimulator.CollisionEngine do
  @moduledoc """
  Core da simulação. Simplificado para apenas detectar colisões e notificar partículas.
  Não publica mais dados para o frontend.
  """
  use GenServer
  require Logger

  alias CollisionSimulator.SpatialHash
  alias CollisionSimulator.Particle
  alias CollisionSimulator.Physics

  # --- Types e Constants (sem alterações) ---
  @typep vec2 :: [float()]
  @typep particle_id :: non_neg_integer()
  @typep batch_states :: %{
           pos: Nx.Tensor.t(),
           vel: Nx.Tensor.t(),
           radius: Nx.Tensor.t(),
           mass: Nx.Tensor.t()
         }
  @typep world_bounds :: %{min_x: float(), max_x: float(), min_y: float(), max_y: float()}
  @type state :: %{world_bounds: world_bounds()}

  @frame_interval_ms 16
  @num_particles 100
  @world_bounds %{x: 0.0, y: 0.0, width: 500, height: 500}
  @particle_radius 5
  @particle_mass 5.0

  def num_particles, do: @num_particles
  def particle_radius, do: @particle_radius
  def particle_mass, do: @particle_mass

  def world_bounds,
    do: %{
      min_x: @world_bounds.x,
      max_x: @world_bounds.x + @world_bounds.width,
      min_y: @world_bounds.y,
      max_y: @world_bounds.y + @world_bounds.height
    }

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  @impl true
  def init(_opts) do
    :ets.new(:particle_data, [
      :set,
      :public,
      :named_table,
      read_concurrency: true,
      write_concurrency: true
    ])

    Process.send_after(self(), :tick, @frame_interval_ms)
    {:ok, %{world_bounds: world_bounds()}}
  end

  @impl true
  def handle_info(:tick, state) do
    start_time = System.monotonic_time()

    detect_and_notify_collisions()

    # NOTA: A chamada para `publish_latest_data()` foi removida daqui.

    elapsed_ms =
      (System.monotonic_time() - start_time) |> System.convert_time_unit(:native, :millisecond)

    next_tick_in_ms = max(@frame_interval_ms - round(elapsed_ms), 0)
    Process.send_after(self(), :tick, next_tick_in_ms)
    {:noreply, state}
  end

  # --- Lógica Principal da Simulação (sem alterações na detecção) ---
  defp detect_and_notify_collisions do
    case :ets.tab2list(:particle_data) do
      [] ->
        :ok

      all_particles ->
        {ids, initial_states} = batch_particles(all_particles)
        spatial_hash = build_spatial_hash(initial_states)
        candidate_pairs = find_candidate_pairs(spatial_hash, initial_states)

        if Enum.any?(candidate_pairs) do
          candidate_tensor = Nx.tensor(candidate_pairs)
          colliding_pairs_tensor = Physics.get_colliding_pairs(initial_states, candidate_tensor)

          if Nx.axis_size(colliding_pairs_tensor, 0) > 0 do
            collision_updates =
              Physics.calculate_collision_responses(initial_states, colliding_pairs_tensor)

            dispatch_collision_updates(ids, collision_updates)
          end
        end
    end

    :ok
  end

  # NOTA: Esta função foi removida.
  # defp publish_latest_data() do ... end

  # --- Funções Auxiliares (sem alterações) ---
  defp find_candidate_pairs(spatial_hash, states) do
    num_particles_in_frame = Nx.axis_size(states.pos, 0)
    positions_list = Nx.to_list(states.pos)
    radii_list = Nx.to_list(states.radius)

    0..(num_particles_in_frame - 1)
    |> Task.async_stream(
      fn index ->
        [px, py] = Enum.at(positions_list, index)
        [r] = Enum.at(radii_list, index)
        r_e = r + 1.0e-6

        SpatialHash.query(spatial_hash, {px, py}, r_e)
        |> Enum.filter(&(&1 > index))
        |> Enum.map(&[index, &1])
      end,
      max_concurrency: System.schedulers_online() * 2,
      ordered: false
    )
    |> Enum.flat_map(fn {:ok, pairs} -> pairs end)
  end

  defp dispatch_collision_updates(ids, collision_updates) do
    ids_tuple = List.to_tuple(ids)

    collision_updates
    |> Task.async_stream(
      fn result ->
        particle_id_a = elem(ids_tuple, result.index_a)
        particle_id_b = elem(ids_tuple, result.index_b)

        GenServer.cast(
          Particle.via_tuple(particle_id_a),
          {:update_after_collision, result.new_vel_a}
        )

        GenServer.cast(
          Particle.via_tuple(particle_id_b),
          {:update_after_collision, result.new_vel_b}
        )
      end,
      timeout: 5000,
      ordered: false
    )
    |> Stream.run()
  end

  defp build_spatial_hash(states) do
    initial_hash = SpatialHash.new(@particle_radius * 2)
    positions_list = Nx.to_list(states.pos)

    Enum.with_index(positions_list)
    |> Enum.reduce(initial_hash, fn {[px, py], index}, acc_hash ->
      SpatialHash.insert(acc_hash, {px, py}, index)
    end)
  end

  defp batch_particles(all_particles) do
    ids = Enum.map(all_particles, &elem(&1, 0))
    pos_idx = Particle.get_attr_index(:pos)
    vel_idx = Particle.get_attr_index(:vel)
    radius_idx = Particle.get_attr_index(:radius)
    mass_idx = Particle.get_attr_index(:mass)

    states = %{
      pos: Enum.map(all_particles, &elem(&1, pos_idx)) |> Nx.tensor(),
      vel: Enum.map(all_particles, &elem(&1, vel_idx)) |> Nx.tensor(),
      radius:
        Enum.map(all_particles, &elem(&1, radius_idx)) |> Nx.tensor() |> Nx.reshape({:auto, 1}),
      mass: Enum.map(all_particles, &elem(&1, mass_idx)) |> Nx.tensor() |> Nx.reshape({:auto, 1})
    }

    {ids, states}
  end
end
