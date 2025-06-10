defmodule CollisionSimulator.CollisionEngine do
  @moduledoc """
  O `CollisionEngine` é o coração da simulação. Ele atua como um orquestrador central
  que opera em um loop de "tick" para impulsionar a detecção de colisões em todo o sistema.

  Responsabilidades principais:
  - Manter um loop de simulação com uma taxa de quadros alvo (frame rate).
  - Em cada "tick", coletar o estado de todas as partículas ativas da tabela ETS.
  - Usar `SpatialHash` para otimizar a detecção, reduzindo o número de pares de partículas a serem verificados.
  - Utilizar o módulo `Physics` (com funções `Nx`) para calcular de forma eficiente quais partículas estão colidindo e quais serão suas novas velocidades.
  - Notificar os processos `Particle` individuais sobre as atualizações de velocidade resultantes das colisões.

  Este módulo foi simplificado para focar exclusivamente na lógica de colisão,
  não sendo mais responsável por publicar dados para o frontend. Essa responsabilidade
  foi delegada aos processos `Particle` individuais.
  """
  use GenServer
  require Logger

  alias CollisionSimulator.SpatialHash
  alias CollisionSimulator.Particle
  alias CollisionSimulator.Physics

  # --- Tipos e Constantes ---

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

  @frame_interval_ms 16 # Aproximadamente 60 quadros por segundo (1000ms / 60fps)
  @num_particles 100
  @world_bounds %{x: 0.0, y: 0.0, width: 500, height: 500}
  @particle_radius 5
  @particle_mass 5.0

  # --- Funções de API Pública ---

  @doc "Retorna o número total de partículas na simulação."
  def num_particles, do: @num_particles

  @doc "Retorna o raio padrão de uma partícula."
  def particle_radius, do: @particle_radius

  @doc "Retorna a massa padrão de uma partícula."
  def particle_mass, do: @particle_mass

  @doc "Retorna os limites do mundo da simulação em um formato de mapa."
  def world_bounds,
    do: %{
      min_x: @world_bounds.x,
      max_x: @world_bounds.x + @world_bounds.width,
      min_y: @world_bounds.y,
      max_y: @world_bounds.y + @world_bounds.height
    }

  @doc "Inicia o GenServer do CollisionEngine."
  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  # --- Callbacks do GenServer ---

  @impl true
  def init(_opts) do
    # Cria uma tabela ETS para armazenar os dados de todas as partículas.
    # :named_table torna-a acessível por seu nome (:particle_data) de outros processos.
    # A concorrência de leitura e escrita é otimizada para alto desempenho.
    :ets.new(:particle_data, [
      :set,
      :public,
      :named_table,
      read_concurrency: true,
      write_concurrency: true
    ])

    # Agenda o primeiro "tick" da simulação.
    Process.send_after(self(), :tick, @frame_interval_ms)
    {:ok, %{world_bounds: world_bounds()}}
  end

  @impl true
  @doc """
  Manipula a mensagem de :tick, que impulsiona o loop principal da simulação.
  """
  def handle_info(:tick, state) do
    start_time = System.monotonic_time()

    # Executa a lógica principal de detecção e notificação de colisões.
    detect_and_notify_collisions()

    # Calcula o tempo gasto neste "tick" para ajustar o próximo.
    # Isso garante que a simulação tente manter uma taxa de quadros constante.
    elapsed_ms =
      (System.monotonic_time() - start_time) |> System.convert_time_unit(:native, :millisecond)

    # Agenda o próximo "tick", subtraindo o tempo já decorrido.
    next_tick_in_ms = max(@frame_interval_ms - round(elapsed_ms), 0)
    Process.send_after(self(), :tick, next_tick_in_ms)

    {:noreply, state}
  end

  # --- Lógica Principal da Simulação ---

  @doc """
  Orquestra o processo de detecção de colisão.
  1. Busca dados de partículas do ETS.
  2. Constrói um `SpatialHash` para otimização.
  3. Encontra pares de candidatos.
  4. Usa `Physics` para confirmar colisões reais.
  5. Calcula as novas velocidades para os pares colididos.
  6. Envia as atualizações para os processos de partículas correspondentes.
  """
  defp detect_and_notify_collisions do
    case :ets.tab2list(:particle_data) do
      [] ->
        # Se não houver partículas, não há nada a fazer.
        :ok

      all_particles ->
        # Prepara os dados para processamento em lote com Nx.
        {ids, initial_states} = batch_particles(all_particles)
        spatial_hash = build_spatial_hash(initial_states)
        candidate_pairs = find_candidate_pairs(spatial_hash, initial_states)

        # Continua somente se houver pares de candidatos a verificar.
        if Enum.any?(candidate_pairs) do
          candidate_tensor = Nx.tensor(candidate_pairs)
          colliding_pairs_tensor = Physics.get_colliding_pairs(initial_states, candidate_tensor)

          # Continua somente se colisões reais forem detectadas.
          if Nx.axis_size(colliding_pairs_tensor, 0) > 0 do
            # Calcula as novas velocidades para os pares que colidiram.
            collision_updates =
              Physics.calculate_collision_responses(initial_states, colliding_pairs_tensor)

            # Envia as atualizações para os processos de partículas individuais.
            dispatch_collision_updates(ids, collision_updates)
          end
        end
    end

    :ok
  end

  # --- Funções Auxiliares ---

  @doc """
  Usa o `SpatialHash` para encontrar pares de partículas que estão próximos o suficiente
  para serem considerados "candidatos" à colisão. Isso otimiza drasticamente o
  desempenho, evitando a verificação de N*N pares.
  A tarefa é paralelizada usando `Task.async_stream`.
  """
  defp find_candidate_pairs(spatial_hash, states) do
    num_particles_in_frame = Nx.axis_size(states.pos, 0)
    positions_list = Nx.to_list(states.pos)
    radii_list = Nx.to_list(states.radius)

    0..(num_particles_in_frame - 1)
    |> Task.async_stream(
      fn index ->
        [px, py] = Enum.at(positions_list, index)
        [r] = Enum.at(radii_list, index)
        # Raio estendido para garantir que não percamos colisões na borda da célula.
        r_e = r + 1.0e-6

        SpatialHash.query(spatial_hash, {px, py}, r_e)
        |> Enum.filter(&(&1 > index)) # Evita pares duplicados (ex: [1,0]) e auto-colisão.
        |> Enum.map(&[index, &1])
      end,
      max_concurrency: System.schedulers_online() * 2,
      ordered: false
    )
    |> Enum.flat_map(fn {:ok, pairs} -> pairs end)
  end

  @doc """
  Envia as novas velocidades calculadas para os processos `Particle` correspondentes
  de forma assíncrona e paralela usando `Task.async_stream`.
  """
  defp dispatch_collision_updates(ids, collision_updates) do
    ids_tuple = List.to_tuple(ids)

    collision_updates
    |> Task.async_stream(
      fn result ->
        # Obtém os IDs reais das partículas a partir de seus índices no lote.
        particle_id_a = elem(ids_tuple, result.index_a)
        particle_id_b = elem(ids_tuple, result.index_b)

        # Envia uma mensagem de `cast` (assíncrona) para cada partícula no par.
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
    |> Stream.run() # Executa o stream.
  end

  @doc """
  Constrói a estrutura de dados `SpatialHash` inserindo a posição de cada partícula.
  """
  defp build_spatial_hash(states) do
    initial_hash = SpatialHash.new(@particle_radius * 2)
    positions_list = Nx.to_list(states.pos)

    Enum.with_index(positions_list)
    |> Enum.reduce(initial_hash, fn {[px, py], index}, acc_hash ->
      SpatialHash.insert(acc_hash, {px, py}, index)
    end)
  end

  @doc """
  Converte a lista de tuplas de partículas do ETS em um mapa de tensores `Nx`,
  que é o formato esperado pelas funções de física. Também retorna uma lista de IDs
  na ordem correspondente.
  """
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
