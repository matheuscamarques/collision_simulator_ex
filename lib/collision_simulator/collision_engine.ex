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

  alias CollisionSimulator.SpatialHash
  alias CollisionSimulator.Particle
  alias CollisionSimulator.Quadtree
  alias CollisionSimulator.Physics

  # --- Types ---
  # Mantendo a consistência com a decisão de usar listas para vetores
  #  @typep vec2 :: [float()]
  @typep particle_id :: non_neg_integer()

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
  @frame_interval_ms 16
  @dt @frame_interval_ms / 1000.0
  @num_particles 20
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

    state = %{
      dt: @dt,
      world_bounds: world_bounds()
    }

    {:ok, state}
  end

  @impl true
  def handle_call(:get_state, _from, state) do
    {:reply, state, state}
  end

  @impl true
  def handle_info(:tick, state) do
    start_time = System.monotonic_time()

    run_simulation_step(state.dt, state.world_bounds)

    elapsed_ms =
      (System.monotonic_time() - start_time)
      |> System.convert_time_unit(:native, :millisecond)

    next_tick_in_ms = max(@frame_interval_ms - round(elapsed_ms), 0)

    Process.send_after(self(), :tick, next_tick_in_ms)
    {:noreply, state}
  end

  # --- Lógica Principal da Simulação ---

  defp run_simulation_step(dt, world_bounds) do
    case :ets.tab2list(:particle_data) do
      [] ->
        :ok

      all_particles ->
        :telemetry.span([:sim, :step], %{}, fn ->
          {ids, initial_states} = batch_particles(all_particles)

          physics_updated_states =
            :telemetry.span([:sim, :physics_update], %{particle_count: @num_particles}, fn ->
              result = Physics.batch_step(initial_states, dt, world_bounds)
              {result, %{}}
            end)

          # quadtree =
          #   :telemetry.span([:sim, :quadtree_build], %{}, fn ->
          #     {build_quadtree(ids, physics_updated_states), %{}}
          #   end)

          # final_states =
          #   :telemetry.span([:sim, :collision_resolution], %{}, fn ->
          #     {resolve_all_collisions(quadtree, physics_updated_states), %{}}
          #   end)

          spatial_hash =
            :telemetry.span([:sim, :spatial_hash_build], %{}, fn ->
              # Usamos os `ids` apenas para manter a consistência, mas a função usa os índices.
              # A função `batch_particles` já retorna os IDs, mas aqui poderíamos usar `0..(@num_particles-1)`
              {build_spatial_hash(physics_updated_states), %{}}
            end)

          final_states =
            :telemetry.span([:sim, :collision_resolution], %{}, fn ->
              {resolve_all_collisions(spatial_hash, physics_updated_states), %{}}
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
  Constrói e popula a SpatialHash a partir das posições atuais das partículas.
  """
  @spec build_spatial_hash(batch_states()) :: SpatialHash.t()
  defp build_spatial_hash(states) do
    # Cria uma nova hash com o tamanho de célula baseado no raio da partícula.
    # Um bom tamanho de célula é ~2x o raio do objeto
    initial_hash = SpatialHash.new(@particle_radius * 2)

    positions_list = Nx.to_list(states.pos)

    # Itera sobre as posições e insere cada partícula na hash.
    Enum.with_index(positions_list)
    |> Enum.reduce(initial_hash, fn {[px, py], index}, acc_hash ->
      # A função query em resolve_all_collisions usa o `index` da partícula, não o `id`.
      # Então, vamos inserir o `index` na hash.
      SpatialHash.insert(acc_hash, {px, py}, index)
    end)
  end

  @doc """
  Converte dados da ETS para tensores otimizados para processamento NX.
  """
  @spec batch_particles(list(tuple())) :: {list(particle_id()), batch_states()}
  def batch_particles(all_particles) do
    ids = Enum.map(all_particles, &elem(&1, 0))

    pos_idx = Particle.get_attr_index(:pos)
    vel_idx = Particle.get_attr_index(:vel)
    radius_idx = Particle.get_attr_index(:radius)
    mass_idx = Particle.get_attr_index(:mass)

    states = %{
      pos:
        Enum.map(all_particles, &elem(&1, pos_idx))
        |> Nx.tensor(),
      vel:
        Enum.map(all_particles, &elem(&1, vel_idx))
        |> Nx.tensor(),
      radius:
        Enum.map(all_particles, &elem(&1, radius_idx))
        |> Nx.tensor()
        |> Nx.reshape({:auto, 1}),
      mass:
        Enum.map(all_particles, &elem(&1, mass_idx))
        |> Nx.tensor()
        |> Nx.reshape({:auto, 1})
    }

    {ids, states}
  end

  @doc """
  Constrói Quadtree a partir das posições atuais das partículas.
  """
  @spec build_quadtree(list(particle_id()), batch_states()) :: Quadtree.t()
  def build_quadtree(ids, states) do
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
  Resolução de colisões em 2 estágios.
  """

  # @spec resolve_all_collisions(Quadtree.t(), batch_states()) :: batch_states()
  # def resolve_all_collisions(quadtree, states) do
  #   num_particles_in_frame = Nx.axis_size(states.pos, 0)
  #   positions_list = Nx.to_list(states.pos)
  #   radii_list = Nx.to_list(states.radius)

  #   candidate_pairs =
  #     0..(num_particles_in_frame - 1)
  #     |> Task.async_stream(
  #       fn index ->
  #         # Esta função agora roda em um processo separado para cada 'index'
  #         [px, py] = Enum.at(positions_list, index)
  #         [r] = Enum.at(radii_list, index)
  #         search_area = %{x: px - r, y: py - r, width: r * 2, height: r * 2}

  #         Quadtree.query(quadtree, search_area)
  #         # Importante: o filtro previne pares duplicados (e.g., [1,5] e [5,1]) e colisões consigo mesmo.
  #         |> Enum.filter(&(&1.index > index))
  #         |> Enum.map(&[index, &1.index])
  #       end,
  #       # Ajuste conforme necessário
  #       max_concurrency: System.schedulers_online() * 2,
  #       ordered: false
  #     )
  #     |> Enum.flat_map(fn {:ok, pairs} -> pairs end)

  #   # O resto da função continua como antes...
  #   if Enum.any?(candidate_pairs) do
  #     candidate_tensor = Nx.tensor(candidate_pairs)
  #     colliding_pairs_tensor = Physics.get_colliding_pairs(states, candidate_tensor)
  #     axis_size = Nx.axis_size(colliding_pairs_tensor, 0)

  #     if axis_size > 0 do
  #       Physics.resolve_all_collisions(states, colliding_pairs_tensor)
  #     else
  #       states
  #     end
  #   else
  #     states
  #   end
  # end

  @spec resolve_all_collisions(SpatialHash.t(), batch_states()) :: batch_states()
  def resolve_all_collisions(spatial_hash, states) do
    num_particles_in_frame = Nx.axis_size(states.pos, 0)
    positions_list = Nx.to_list(states.pos)
    radii_list = Nx.to_list(states.radius)

    candidate_pairs =
      0..(num_particles_in_frame - 1)
      |> Task.async_stream(
        fn index ->
          # Pega a posição e o raio da partícula atual
          [px, py] = Enum.at(positions_list, index)
          [r] = Enum.at(radii_list, index)

          # Consulta o Spatial Hash para encontrar vizinhos próximos
          SpatialHash.query(spatial_hash, {px, py}, r)
          # Filtra para evitar pares duplicados (ex: [1,5] e [5,1]) e colisões da partícula consigo mesma.
          |> Enum.filter(&(&1 > index))
          |> Enum.map(&[index, &1])
        end,
        max_concurrency: System.schedulers_online() * 2,
        ordered: false
      )
      |> Enum.flat_map(fn {:ok, pairs} -> pairs end)

    # O resto da função continua exatamente como antes...
    if Enum.any?(candidate_pairs) do
      candidate_tensor = Nx.tensor(candidate_pairs)
      colliding_pairs_tensor = Physics.get_colliding_pairs(states, candidate_tensor)
      axis_size = Nx.axis_size(colliding_pairs_tensor, 0)

      if axis_size > 0 do
        Physics.resolve_all_collisions(states, colliding_pairs_tensor)
      else
        states
      end
    else
      states
    end
  end

  @doc """
  Atualiza a tabela ETS em lote com os estados finais das partículas.
  """
  @spec update_ets_batch(list(particle_id()), batch_states()) :: :ok
  def update_ets_batch(ids, states) do
    positions = Nx.to_list(states.pos)
    velocities = Nx.to_list(states.vel)
    radii = Nx.to_list(states.radius)
    masses = Nx.to_list(states.mass)

    # CORREÇÃO: Usando Enum.zip/1 para combinar as cinco listas.
    # Esta é a forma correta e idiomática de fazer o "zip" de múltiplas listas.
    zipped_data =
      Enum.zip([ids, positions, velocities, radii, masses])
      |> Enum.map(fn {id, pos, vel, [radius], [mass]} ->
        # Desestruturando os valores para criar a tupla final
        {id, pos, vel, radius, mass}
      end)

    # Insere todos os dados em uma única operação otimizada
    :ets.insert(:particle_data, zipped_data)

    :ok
  end

  @doc """
  Publica os dados da simulação para visualização.
  """
  @spec publish_particle_data(batch_states()) :: :ok
  defp publish_particle_data(%{pos: pos, radius: radius} = _final_states) do
    payload = %{
      positions: Nx.to_list(pos),
      radii: Nx.to_list(radius) |> List.flatten()
    }

    Phoenix.PubSub.broadcast(
      CollisionSimulator.PubSub,
      "simulation_updates",
      {:particle_data, payload}
    )
  end
end
