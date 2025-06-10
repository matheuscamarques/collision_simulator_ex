defmodule CollisionSimulator.CollisionEngine do
  @moduledoc """
  Core da simulação: GenServer que detecta colisões e notifica as partículas.

  Arquitetura Refatorada:
  - Atua como um serviço de detecção, não como o controlador de estado.
  - O loop principal (`tick`) lê o estado da ETS para ter uma visão global.
  - Usa Nx e SpatialHash para encontrar pares de partículas em colisão.
  - Calcula as velocidades resultantes da colisão.
  - Envia mensagens (`cast`) para as partículas envolvidas atualizarem seus próprios estados.
  - Publica os dados da ETS para o frontend.
  """
  use GenServer
  require Logger

  alias CollisionSimulator.SpatialHash
  alias CollisionSimulator.Particle
  alias CollisionSimulator.Physics

  # --- Types ---
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

  # --- Constants ---
  @frame_interval_ms 16
  @num_particles 50
  @world_bounds %{x: 0.0, y: 0.0, width: 500, height: 500}
  @particle_radius 5
  @particle_mass 5.0

  # --- Funções Públicas ---
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

  # --- API do Cliente ---
  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  # --- Callbacks do GenServer ---
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
    state = %{world_bounds: world_bounds()}
    {:ok, state}
  end

  @impl true
  def handle_info(:tick, state) do
    start_time = System.monotonic_time()

    # Executa o passo de detecção e notificação
    detect_and_notify_collisions()

    # Publica os dados mais recentes da ETS para o frontend
    publish_latest_data()

    elapsed_ms =
      (System.monotonic_time() - start_time)
      |> System.convert_time_unit(:native, :millisecond)

    next_tick_in_ms = max(@frame_interval_ms - round(elapsed_ms), 0)
    Process.send_after(self(), :tick, next_tick_in_ms)
    {:noreply, state}
  end

  # --- Lógica Principal da Simulação ---

  defp detect_and_notify_collisions do
    case :ets.tab2list(:particle_data) do
      [] ->
        :ok

      all_particles ->
        # Telemetria para o passo completo de detecção
        :telemetry.span([:sim, :detection_step], %{}, fn ->
          {ids, initial_states} = batch_particles(all_particles)

          spatial_hash =
            :telemetry.span([:sim, :spatial_hash_build], %{}, fn ->
              {build_spatial_hash(initial_states), %{}}
            end)

          candidate_pairs = find_candidate_pairs(spatial_hash, initial_states)

          if Enum.any?(candidate_pairs) do
            candidate_tensor = Nx.tensor(candidate_pairs)
            colliding_pairs_tensor = Physics.get_colliding_pairs(initial_states, candidate_tensor)

            if Nx.axis_size(colliding_pairs_tensor, 0) > 0 do
              # Calcula as respostas de colisão (novas velocidades)
              collision_updates =
                :telemetry.span([:sim, :collision_calculation], %{}, fn ->
                  {Physics.calculate_collision_responses(initial_states, colliding_pairs_tensor),
                   %{}}
                end)

              # Envia as atualizações para as partículas envolvidas
              :telemetry.span([:sim, :dispatch_notifications], %{}, fn ->
                {dispatch_collision_updates(ids, collision_updates), %{}}
              end)
            end
          end

          {:ok, %{}}
        end)
    end

    :ok
  end

  defp find_candidate_pairs(spatial_hash, states) do
    num_particles_in_frame = Nx.axis_size(states.pos, 0)
    positions_list = Nx.to_list(states.pos)
    radii_list = Nx.to_list(states.radius)

    0..(num_particles_in_frame - 1)
    |> Task.async_stream(
      fn index ->
        [px, py] = Enum.at(positions_list, index)
        [r] = Enum.at(radii_list, index)
        # r + slmall_epsilon para evitar colisões de partículas com raio zero
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
    # Etapa 1: Otimização de acesso. Converter para tupla para acesso O(1).
    # Isso é feito uma vez fora do loop.
    ids_tuple = List.to_tuple(ids)

    # Etapa 2: Usar Task.async_stream para despacho em paralelo.
    collision_updates
    |> Task.async_stream(
      fn result ->
        # Agora usamos elem/2, que é instantâneo.
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
      # Não precisamos de um timeout longo, pois o trabalho é muito rápido.
      timeout: 5000,
      # Não nos importamos com a ordem dos resultados.
      ordered: false
    )
    # Usamos Stream.run() para consumir a stream e garantir que todas as tarefas sejam executadas.
    |> Stream.run()
  end

  defp publish_latest_data() do
    # Lê os dados mais recentes da ETS, que foram atualizados pelas próprias partículas.
    all_particles = :ets.tab2list(:particle_data)

    payload = %{
      positions: Enum.map(all_particles, &elem(&1, Particle.get_attr_index(:pos))),
      radii: Enum.map(all_particles, &elem(&1, Particle.get_attr_index(:radius)))
    }

    Phoenix.PubSub.broadcast(
      CollisionSimulator.PubSub,
      "simulation_updates",
      {:particle_data, payload}
    )
  end

  # --- Funções Auxiliares (sem alterações significativas) ---

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
