defmodule CollisionSimulator.CollisionEngine do
  @moduledoc """
  Core da simulação: GenServer que orquestra o ciclo completo de atualização física.

  Arquitetura:
  - Processamento em lote (batch) usando Nx tensors
  - Pipeline otimizado com telemetria
  - Tabela ETS compartilhada para estado das partículas

  Fluxo de execução (por frame):
  1. Coleta estados da ETS -> Conversão para tensores
  2. Atualização física básica (movimento)
  3. Construção da Quadtree
  4. Detecção de colisões
  5. Resolução de colisões
  6. Atualização da ETS
  7. Broadcast via PubSub

  Otimizações-chave:
  - Operações vetorizadas com Nx
  - Quadtree para redução O(n²) -> O(n log n)
  - Controle de taxa de frames adaptativo
  """
  use GenServer
  require Logger

  alias CollisionSimulator.Quadtree
  alias CollisionSimulator.Physics

  # --- Types ---
  @typep vec2 :: {float(), float()}
  @typep particle_id :: non_neg_integer()

  @typep particle_state :: %{
           id: particle_id(),
           pos: vec2(),
           vel: vec2(),
           radius: float(),
           mass: float()
         }

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

  @type state :: %{
          dt: float(),
          world_bounds: world_bounds()
        }

  # --- Constants ---

  # Intervalo de frame alvo em milissegundos (ex: 16ms para ~60 FPS).
  @frame_interval_ms 16
  # Delta time para os cálculos de física, em segundos.
  @dt @frame_interval_ms / 1000.0
  # Número total de partículas na simulação.
  @num_particles 20
  # As fronteiras do mundo da simulação.
  @world_bounds %{x: 0.0, y: 0.0, width: 500, height: 500}
  # Raio padrão para cada partícula.
  @particle_radius 5.0
  # Massa padrão para cada partícula.
  @particle_mass 1.0

  # --- Funções Públicas ---
  @spec num_particles() :: non_neg_integer()
  def num_particles, do: @num_particles

  @spec particle_radius() :: float()
  def particle_radius, do: @particle_radius

  @spec particle_mass() :: float()
  def particle_mass, do: @particle_mass

  @spec world_bounds() :: world_bounds()
  def world_bounds,
    do: %{
      min_x: @world_bounds.x,
      max_x: @world_bounds.x + @world_bounds.width,
      min_y: @world_bounds.y,
      max_y: @world_bounds.y + @world_bounds.height
    }

  # --- API do Cliente ---
  @spec start_link(any()) :: GenServer.on_start()
  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  # --- Callbacks do GenServer ---
  @impl true
  @spec init(any()) :: {:ok, state()}
  def init(_opts) do
    # Cria a tabela ETS para armazenar os dados das partículas.
    # Isso deve ser feito antes que o ParticleSupervisor inicie seus filhos.
    :ets.new(:particle_data, [
      :set,
      :public,
      :named_table,
      read_concurrency: true,
      write_concurrency: true
    ])

    # Agenda o primeiro "tick" da simulação.
    Process.send_after(self(), :tick, @frame_interval_ms)

    state = %{
      dt: @dt,
      world_bounds: world_bounds()
    }

    {:ok, state}
  end

  @impl true
  @spec handle_info(:tick, state()) :: {:noreply, state()}
  def handle_info(:tick, state) do
    # Melhoria: Controle de taxa de atualização para uma simulação mais estável.
    start_time = System.monotonic_time()

    run_simulation_step(state.dt, state.world_bounds)

    elapsed_ms =
      (System.monotonic_time() - start_time) |> System.convert_time_unit(:native, :millisecond)

    next_tick_in_ms = max(@frame_interval_ms - round(elapsed_ms), 0)

    # Agenda o próximo tick.
    Process.send_after(self(), :tick, next_tick_in_ms)
    {:noreply, state}
  end

  # --- Lógica Principal da Simulação ---

  @spec run_simulation_step(float(), world_bounds()) :: :ok
  defp run_simulation_step(dt, world_bounds) do
    case :ets.tab2list(:particle_data) do
      [] ->
        :ok

      all_particles ->
        :telemetry.span([:sim, :step], %{}, fn ->
          {ids, initial_states} = batch_particles(all_particles)

          physics_updated_states =
            :telemetry.span([:sim, :physics_update], %{particle_count: @num_particles}, fn ->
              # IO.inspect("Running physics update for #{@num_particles} particles")
              result = Physics.batch_step(initial_states, dt, world_bounds)
              {result, %{}}
            end)

          quadtree =
            :telemetry.span([:sim, :quadtree_build], %{}, fn ->
              {build_quadtree(ids, physics_updated_states), %{}}
            end)

          final_states =
            :telemetry.span([:sim, :collision_resolution], %{}, fn ->
              {resolve_all_collisions(quadtree, physics_updated_states), %{}}
            end)

          :telemetry.span([:sim, :ets_update], %{}, fn ->
            {update_ets_batch(ids, final_states), %{}}
          end)

          :telemetry.span([:sim, :publish], %{}, fn ->
            {publish_particle_data(final_states), %{}}
          end)

          {:ok, %{}}
        end)
    end

    :ok
  end

  @doc """
  Converte dados da ETS para tensores otimizados para processamento NX.

  Parâmetros:
  - all_particles: Lista de tuplas {id, state} da ETS

  Retorna:
  - {ids, tensor_states} onde tensor_states é um mapa com chaves:
    * :pos -> tensor [num_particles, 2]
    * :vel -> tensor [num_particles, 2]
    * :radius -> tensor [num_particles, 1]
    * :mass -> tensor [num_particles, 1]
  """

  @spec batch_particles(list({particle_id(), particle_state()})) ::
          {list(particle_id()), batch_states()}
  def batch_particles(all_particles) do
    ids = Enum.map(all_particles, &elem(&1, 0))

    # Função auxiliar para garantir lista de floats
    to_list = fn
      tuple when is_tuple(tuple) -> Tuple.to_list(tuple)
      list when is_list(list) -> list
    end

    states = %{
      pos:
        Enum.map(all_particles, &to_list.(elem(&1, 1).pos))
        |> Nx.tensor(),
      vel:
        Enum.map(all_particles, &to_list.(elem(&1, 1).vel))
        |> Nx.tensor(),
      radius:
        Enum.map(all_particles, &elem(&1, 1).radius)
        |> Nx.tensor()
        |> Nx.reshape({:auto, 1}),
      mass:
        Enum.map(all_particles, &elem(&1, 1).mass)
        |> Nx.tensor()
        |> Nx.reshape({:auto, 1})
    }

    {ids, states}
  end

  @doc """
  Constrói Quadtree a partir das posições atuais das partículas.

  Estratégia:
  - Cada partícula é representada como retângulo delimitador (AABB)
  - Árvore é reconstruída a cada frame
  - Otimizada para consultas espaciais rápidas
  """
  @spec build_quadtree(list(particle_id()), batch_states()) :: Quadtree.t()
  def build_quadtree(ids, states) do
    # Correção: Usando o atributo de módulo para as fronteiras, que tem o formato esperado.
    initial_quadtree = Quadtree.create(width: @world_bounds.width, height: @world_bounds.height)
    positions_list = Nx.to_list(states.pos)
    radii_list = Nx.to_list(states.radius)

    positions_list
    |> Enum.with_index()
    |> Enum.reduce(initial_quadtree, fn {[px, py], index}, acc_qt ->
      id = Enum.at(ids, index)
      [r] = Enum.at(radii_list, index)
      rect = %{id: id, index: index, x: px - r, y: py - r, width: r * 2, height: r * 2}
      Quadtree.insert(acc_qt, rect)
    end)
  end

  @doc """
  Resolução de colisões em 2 estágios:
  1. Detecção: Usa Quadtree para identificar pares candidatos
  2. Filtragem: Verificação precisa de colisão círculo-círculo

  Retorna estados atualizados com física de colisão aplicada
  """
  @spec resolve_all_collisions(Quadtree.t(), batch_states()) :: batch_states()
  def resolve_all_collisions(quadtree, states) do
    # Correção: Usar a contagem real de partículas do lote atual, não a constante.
    num_particles_in_frame = Nx.axis_size(states.pos, 0)

    candidate_pairs =
      0..(num_particles_in_frame - 1)
      |> Enum.flat_map(fn index ->
        pos_tensor = states.pos[index]
        [px, py] = Nx.to_list(pos_tensor)
        [r] = Nx.to_list(states.radius[index])
        search_area = %{x: px - r, y: py - r, width: r * 2, height: r * 2}

        Quadtree.query(quadtree, search_area)
        |> Enum.filter(&(&1.index > index))
        |> Enum.map(&[index, &1.index])
      end)

    if Enum.any?(candidate_pairs) do
      candidate_tensor = Nx.tensor(candidate_pairs)

      colliding_pairs_tensor = Physics.get_colliding_pairs(states, candidate_tensor)
      axis_size = Nx.axis_size(colliding_pairs_tensor, 0)
      # Logger.info("Found #{axis_size} colliding pairs")

      if axis_size > 0 do
        Physics.resolve_all_collisions(states, colliding_pairs_tensor)
      else
        states
      end
    else
      states
    end
  end

  @spec update_ets_batch(list(particle_id()), batch_states()) :: :ok
  @spec update_ets_batch(list(particle_id()), batch_states()) :: :ok
  def update_ets_batch(ids, states) do
    positions = Nx.to_list(states.pos)
    velocities = Nx.to_list(states.vel)
    radii = Nx.to_list(states.radius)
    masses = Nx.to_list(states.mass)

    zipped =
      Enum.zip([ids, positions, velocities, radii, masses])

    zipped
    # ajustável: depende do seu workload
    |> Stream.chunk_every(500)
    |> Task.async_stream(
      fn chunk ->
        chunk
        |> Enum.map(fn {id, [x, y], [vx, vy], [r], [m]} ->
          {id,
           %{
             id: id,
             pos: {x, y},
             vel: {vx, vy},
             radius: r,
             mass: m
           }}
        end)
        |> then(&:ets.insert(:particle_data, &1))
      end,
      max_concurrency: System.schedulers_online(),
      timeout: :infinity
    )
    |> Stream.run()

    :ok
  end

  @spec publish_particle_data(batch_states()) :: :ok
  defp publish_particle_data(%{pos: pos} = _final_states) do
    num_particles = Nx.axis_size(pos, 0)

    payload = %{
      positions: Nx.to_list(pos),
      radii: :lists.duplicate(num_particles, @particle_radius)
    }

    Phoenix.PubSub.broadcast(
      CollisionSimulator.PubSub,
      "simulation_updates",
      {:particle_data, payload}
    )
  end
end
