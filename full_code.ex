defmodule CollisionSimulator.CollisionEngine do
  @moduledoc """
  O motor de simulação principal, executado como um GenServer central.

  Este módulo orquestra todo o ciclo da simulação. Foi refatorado
  para usar um modelo de processamento em lote (batch), que é significativamente mais
  performático que o modelo anterior de um GenServer por partícula. Ele gerencia o
  loop da simulação, coordena os cálculos de física, a detecção de colisões
  e publica os resultados para a interface do usuário.
  """
  use GenServer
  require Logger

  alias CollisionSimulator.Quadtree
  alias CollisionSimulator.Physics

  # --- Tipos ---
  # Define tipos customizados para clareza e uso com @spec.
  @typep vec2 :: {float(), float()}
  @typep particle_id :: non_neg_integer()

  # Estado de uma única partícula.
  @typep particle_state :: %{
           id: particle_id(),
           pos: vec2(),
           vel: vec2(),
           radius: float(),
           mass: float()
         }

  # Estrutura que agrupa os estados de todas as partículas em tensores Nx.
  @typep batch_states :: %{
           pos: Nx.Tensor.t(),
           vel: Nx.Tensor.t(),
           radius: Nx.Tensor.t(),
           mass: Nx.Tensor.t()
         }

  # Define as fronteiras do mundo da simulação.
  @typep world_bounds :: %{
           min_x: float(),
           max_x: float(),
           min_y: float(),
           max_y: float()
         }

  # O estado interno do GenServer do motor.
  @type state :: %{
          dt: float(),
          world_bounds: world_bounds()
        }

  # --- Constantes ---

  # Intervalo de frame alvo em milissegundos (ex: 16ms para ~60 FPS).
  @frame_interval_ms 16
  # Delta time para os cálculos de física, em segundos.
  @dt @frame_interval_ms / 1000.0
  # Número total de partículas na simulação.
  @num_particles 20
  # As fronteiras do mundo da simulação.
  @world_bounds %{x: 0.0, y: 0.0, width: 800.0, height: 600.0}
  # Raio padrão para cada partícula.
  @particle_radius 5.0
  # Massa padrão para cada partícula.
  @particle_mass 1.0

  # --- Funções Públicas ---
  # Funções de acesso para que outros módulos possam obter as constantes da simulação.
  @doc "Retorna o número de partículas configurado para a simulação."
  @spec num_particles() :: non_neg_integer()
  def num_particles, do: @num_particles

  @doc "Retorna o raio padrão das partículas."
  @spec particle_radius() :: float()
  def particle_radius, do: @particle_radius

  @doc "Retorna a massa padrão das partículas."
  @spec particle_mass() :: float()
  def particle_mass, do: @particle_mass

  @doc "Retorna o mapa com as fronteiras do mundo."
  @spec world_bounds() :: world_bounds()
  def world_bounds,
    do: %{
      min_x: @world_bounds.x,
      max_x: @world_bounds.x + @world_bounds.width,
      min_y: @world_bounds.y,
      max_y: @world_bounds.y + @world_bounds.height
    }

  # --- API do Cliente ---
  @doc "Inicia o GenServer do motor de simulação."
  @spec start_link(any()) :: GenServer.on_start()
  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  # --- Callbacks do GenServer ---
  @impl true
  @doc "Inicializa o estado do motor de simulação."
  @spec init(any()) :: {:ok, state()}
  def init(_opts) do
    # Cria a tabela ETS para armazenar os dados das partículas.
    # O ETS (Erlang Term Storage) oferece acesso concorrente de alta performance
    # aos dados, o que é crucial para a arquitetura em lote.
    # O `ParticleSupervisor` depende que esta tabela exista antes de iniciar seus filhos.
    :ets.new(:particle_data, [
      :set,
      :public,
      :named_table,
      read_concurrency: true,
      write_concurrency: true
    ])

    # Agenda o primeiro "tick" da simulação para iniciar o loop.
    Process.send_after(self(), :tick, @frame_interval_ms)

    # Define o estado inicial do motor.
    state = %{
      dt: @dt,
      world_bounds: world_bounds()
    }

    {:ok, state}
  end

  @impl true
  @doc "Lida com a mensagem de 'tick', executando um passo da simulação."
  @spec handle_info(:tick, state()) :: {:noreply, state()}
  def handle_info(:tick, state) do
    # Mede o tempo de execução do passo da simulação para estabilizar a taxa de quadros.
    start_time = System.monotonic_time()

    run_simulation_step(state.dt, state.world_bounds)

    # Calcula quanto tempo o passo levou.
    elapsed_ms =
      (System.monotonic_time() - start_time) |> System.convert_time_unit(:native, :millisecond)

    # Garante uma taxa de quadros estável. Se a simulação for mais rápida que o
    # intervalo de frame, ele espera. Se for mais lenta, agenda o próximo tick imediatamente.
    next_tick_in_ms = max(@frame_interval_ms - round(elapsed_ms), 0)

    # Agenda o próximo tick.
    Process.send_after(self(), :tick, next_tick_in_ms)
    {:noreply, state}
  end

  # --- Lógica Principal da Simulação ---

  @doc "Executa um único passo completo do pipeline da simulação."
  @spec run_simulation_step(float(), world_bounds()) :: :ok
  defp run_simulation_step(dt, world_bounds) do
    # Lê todos os dados das partículas da tabela ETS.
    case :ets.tab2list(:particle_data) do
      # Se não houver partículas, não faz nada.
      [] ->
        :ok

      all_particles ->
        # Utiliza Telemetry para medir a performance de cada etapa do pipeline.
        :telemetry.span([:sim, :step], %{}, fn ->
          # 1. Agrupa os dados das partículas em tensores Nx.
          {ids, initial_states} = batch_particles(all_particles)

          # 2. Executa o passo de física (movimento e colisão com paredes).
          physics_updated_states =
            :telemetry.span([:sim, :physics_update], %{particle_count: @num_particles}, fn ->
              result = Physics.batch_step(initial_states, dt, world_bounds)
              {result, %{}}
            end)

          # 3. Constrói a Quadtree para otimizar a detecção de colisões.
          quadtree =
            :telemetry.span([:sim, :quadtree_build], %{}, fn ->
              {build_quadtree(ids, physics_updated_states), %{}}
            end)

          # 4. Detecta e resolve colisões entre partículas.
          final_states =
            :telemetry.span([:sim, :collision_resolution], %{}, fn ->
              {resolve_all_collisions(quadtree, physics_updated_states), %{}}
            end)

          # 5. Atualiza a tabela ETS com os novos estados das partículas.
          :telemetry.span([:sim, :ets_update], %{}, fn ->
            {update_ets_batch(ids, final_states), %{}}
          end)

          # 6. Publica os dados para os assinantes (e.g., a interface do LiveView).
          :telemetry.span([:sim, :publish], %{}, fn ->
            {publish_particle_data(final_states), %{}}
          end)

          {:ok, %{}}
        end)
    end

    :ok
  end

  # --- Funções Auxiliares do Pipeline ---

  @doc "Converte uma lista de estados de partículas do ETS em um mapa de tensores Nx."
  @spec batch_particles(list({particle_id(), particle_state()})) ::
          {list(particle_id()), batch_states()}
  defp batch_particles(all_particles) do
    # Extrai os IDs para manter a correspondência com os tensores.
    ids = Enum.map(all_particles, &elem(&1, 0))

    # Função auxiliar para garantir que os dados de vetores sejam listas.
    to_list = fn
      tuple when is_tuple(tuple) -> Tuple.to_list(tuple)
      list when is_list(list) -> list
    end

    # Cria tensores para posições, velocidades, raios e massas.
    # Esta estrutura de dados é otimizada para os cálculos vetorizados do Nx.
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
        # Garante que seja um tensor coluna.
        |> Nx.reshape({:auto, 1}),
      mass:
        Enum.map(all_particles, &elem(&1, 1).mass)
        |> Nx.tensor()
        # Garante que seja um tensor coluna.
        |> Nx.reshape({:auto, 1})
    }

    {ids, states}
  end

  @doc "Constrói uma Quadtree a partir dos estados atuais das partículas."
  @spec build_quadtree(list(particle_id()), batch_states()) :: Quadtree.t()
  defp build_quadtree(ids, states) do
    # Cria uma Quadtree vazia com as dimensões do mundo.
    initial_quadtree = Quadtree.create(width: @world_bounds.width, height: @world_bounds.height)
    positions_list = Nx.to_list(states.pos)
    radii_list = Nx.to_list(states.radius)

    # Itera sobre cada partícula e a insere na Quadtree.
    # A Quadtree armazena a caixa delimitadora (bounding box) de cada partícula.
    positions_list
    |> Enum.with_index()
    |> Enum.reduce(initial_quadtree, fn {[px, py], index}, acc_qt ->
      id = Enum.at(ids, index)
      [r] = Enum.at(radii_list, index)
      # Cria um retângulo representando a caixa delimitadora da partícula.
      rect = %{id: id, index: index, x: px - r, y: py - r, width: r * 2, height: r * 2}
      Quadtree.insert(acc_qt, rect)
    end)
  end

  @doc "Usa a Quadtree para encontrar e resolver todas as colisões entre partículas."
  @spec resolve_all_collisions(Quadtree.t(), batch_states()) :: batch_states()
  defp resolve_all_collisions(quadtree, states) do
    # Usa a contagem real de partículas no frame, que pode ser diferente da constante.
    num_particles_in_frame = Nx.axis_size(states.pos, 0)

    # 1. Encontra pares de partículas candidatas à colisão.
    candidate_pairs =
      0..(num_particles_in_frame - 1)
      |> Enum.flat_map(fn index ->
        # Para cada partícula, consulta a Quadtree para encontrar outras próximas.
        pos_tensor = states.pos[index]
        [px, py] = Nx.to_list(pos_tensor)
        [r] = Nx.to_list(states.radius[index])
        search_area = %{x: px - r, y: py - r, width: r * 2, height: r * 2}

        Quadtree.query(quadtree, search_area)
        # Filtra para evitar pares duplicados (e.g., [1,2] e [2,1]) e colisões consigo mesmo.
        |> Enum.filter(&(&1.index > index))
        |> Enum.map(&[index, &1.index])
      end)

    if Enum.any?(candidate_pairs) do
      candidate_tensor = Nx.tensor(candidate_pairs)

      # 2. Filtra os pares candidatos para encontrar apenas aqueles que realmente colidem.
      colliding_pairs_tensor = Physics.get_colliding_pairs(states, candidate_tensor)
      axis_size = Nx.axis_size(colliding_pairs_tensor, 0)

      # 3. Se houver colisões, resolve-as.
      if axis_size > 0 do
        Physics.resolve_all_collisions(states, colliding_pairs_tensor)
      else
        # Retorna os estados inalterados se não houver colisões.
        states
      end
    else
      # Retorna os estados inalterados se não houver candidatos.
      states
    end
  end

  @doc "Atualiza a tabela ETS com os novos estados das partículas de forma otimizada."
  @spec update_ets_batch(list(particle_id()), batch_states()) :: :ok
  defp update_ets_batch(ids, final_states) do
    # Converte os tensores de volta para listas de listas.
    positions_list = Nx.to_list(final_states.pos)
    velocities_list = Nx.to_list(final_states.vel)

    # Prepara uma lista de tuplas {id, estado} para inserção em lote no ETS.
    updates =
      Enum.zip(ids, Enum.zip(positions_list, velocities_list))
      |> Enum.map(fn {id, {pos_list, vel_list}} ->
        state = %{
          id: id,
          pos: List.to_tuple(pos_list),
          vel: List.to_tuple(vel_list),
          radius: @particle_radius,
          mass: @particle_mass
        }

        {id, state}
      end)

    # A inserção em lote (:ets.insert com uma lista) é muito mais eficiente que inserções individuais.
    :ets.insert(:particle_data, updates)
    :ok
  end

  @doc "Publica os dados de posição e raio das partículas via Phoenix.PubSub."
  @spec publish_particle_data(batch_states()) :: :ok
  defp publish_particle_data(final_states) do
    num_particles_in_frame = Nx.axis_size(final_states.pos, 0)
    radii = List.duplicate(@particle_radius, num_particles_in_frame)

    # Prepara o payload com os dados que a interface precisa para renderizar.
    payload = %{
      positions: Nx.to_list(final_states.pos),
      radii: radii
    }

    # Transmite os dados para o tópico "simulation_updates".
    # O LiveView da simulação estará inscrito neste tópico para receber as atualizações.
    Phoenix.PubSub.broadcast(
      CollisionSimulator.PubSub,
      "simulation_updates",
      {:particle_data, payload}
    )
  end
end

defmodule CollisionSimulator.ParticleSupervisor do
  @moduledoc """
  Um supervisor responsável por iniciar e gerenciar todos os workers de `Particle`.
  Seu principal papel é criar o número desejado de partículas com posições
  e velocidades iniciais aleatórias quando a aplicação começa.
  """
  use Supervisor

  @doc "Inicia o supervisor de partículas."
  @spec start_link(any()) :: Supervisor.on_start()
  def start_link(init_arg) do
    Supervisor.start_link(__MODULE__, init_arg, name: __MODULE__)
  end

  @impl true
  @doc "Define a estratégia de supervisão e os filhos a serem iniciados."
  @spec init(any()) :: {:ok, Supervisor.sup_flags()}
  def init(_init_arg) do
    # Obtém as constantes da simulação do módulo CollisionEngine.
    num_particles = CollisionSimulator.CollisionEngine.num_particles()
    bounds = CollisionSimulator.CollisionEngine.world_bounds()
    radius = CollisionSimulator.CollisionEngine.particle_radius()
    mass = CollisionSimulator.CollisionEngine.particle_mass()

    # Gera a especificação de filho para cada partícula.
    children =
      Enum.map(0..(num_particles - 1), fn i ->
        # Gera uma posição aleatória dentro das fronteiras do mundo.
        pos = [
          random_in_range(bounds.min_x + radius, bounds.max_x - radius),
          random_in_range(bounds.min_y + radius, bounds.max_y - radius)
        ]

        # Gera uma velocidade aleatória.
        vel = [Enum.random(-50..50) * 1.0, Enum.random(-50..50) * 1.0]

        # Cada filho é uma especificação de worker para um GenServer Particle.
        # Ele será iniciado com um estado inicial único.
        %{
          id: i,
          start:
            {CollisionSimulator.Particle, :start_link,
             [[id: i, pos: pos, vel: vel, radius: radius, mass: mass]]},
          # Reinicia a partícula se ela falhar.
          restart: :permanent,
          type: :worker
        }
      end)

    # Inicia os filhos sob a estratégia :one_for_one, onde a falha de um
    # filho não afeta os outros.
    Supervisor.init(children, strategy: :one_for_one)
  end

  @doc "Gera um número de ponto flutuante aleatório dentro de um intervalo."
  @spec random_in_range(number(), number()) :: float()
  defp random_in_range(min, max), do: :rand.uniform() * (max - min) + min
end

defmodule CollisionSimulator.Particle do
  @moduledoc """
  Representa uma única partícula na simulação.

  Nesta arquitetura refatorada, o `Particle` GenServer é primariamente um
  guardião de estado. Seu principal trabalho na inicialização é registrar seu estado inicial
  na tabela ETS central. Todos os cálculos de física foram movidos para os
  módulos `CollisionEngine` e `Physics` para processamento em lote eficiente.
  """
  use GenServer

  # --- Tipos ---
  @type id :: non_neg_integer()
  @type vec2 :: {float(), float()}
  @type state :: %{
          id: id(),
          pos: vec2(),
          vel: vec2(),
          radius: float(),
          mass: float()
        }

  # --- API do Cliente ---
  @doc "Inicia o GenServer da partícula e o registra."
  @spec start_link(list()) :: GenServer.on_start()
  def start_link(opts) do
    id = Keyword.fetch!(opts, :id)
    # Usa `via_tuple` para registrar o processo com um nome baseado em seu ID.
    GenServer.start_link(__MODULE__, opts, name: via_tuple(id))
  end

  @doc "Cria uma tupla `{:via, ...}` para registrar o processo no Registry."
  @spec via_tuple(id()) :: {:via, module(), {module(), id()}}
  def via_tuple(id), do: {:via, Registry, {CollisionSimulator.ParticleRegistry, id}}

  # --- Callbacks do Servidor ---
  @impl true
  @doc "Inicializa o estado da partícula e o insere na tabela ETS."
  @spec init(list()) :: {:ok, state()}
  def init(opts) do
    # Monta o estado inicial a partir das opções recebidas.
    state = %{
      id: Keyword.fetch!(opts, :id),
      pos: Keyword.fetch!(opts, :pos),
      vel: Keyword.fetch!(opts, :vel),
      radius: Keyword.fetch!(opts, :radius),
      mass: Keyword.fetch!(opts, :mass)
    }

    # Na inicialização, a partícula insere seu próprio estado na tabela ETS.
    # Esta é a única vez que ela interage diretamente com o ETS.
    # A partir daqui, o CollisionEngine gerencia seu estado.
    :ets.insert(:particle_data, {state.id, state})
    {:ok, state}
  end
end

defmodule CollisionSimulator.Physics do
  @moduledoc """
  Contém todos os cálculos de física orientados a lote usando `Nx.Defn`.

  Este módulo é projetado para operar em tensores que representam todo o conjunto de
  partículas, o que é altamente eficiente. As funções dentro deste módulo (`defn`) são
  compiladas Just-In-Time (JIT) pelo Nx (usando EXLA como backend) para
  execução em CPU ou GPU, resultando em um desempenho máximo.
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
  Filtra uma lista de pares candidatos para retornar apenas aqueles que estão realmente colidindo.
  Esta função é vetorizada: em vez de um loop, ela opera em todos os pares de uma vez.
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
  Resolve todas as colisões para um dado lote de pares colidindo.
  Isso envolve duas etapas: correção de posição para evitar que as partículas
  se sobreponham e resolução de velocidade para simular o impulso da colisão.
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

defmodule CollisionSimulator.Quadtree do
  @moduledoc """
  Implementação de uma Quadtree para particionamento espacial.

  Uma Quadtree é uma estrutura de dados em árvore na qual cada nó interno
  tem exatamente quatro filhos. Ela é usada para particionar um espaço 2D,
  subdividindo-o recursivamente em quatro quadrantes. O objetivo é otimizar
  a detecção de colisões, evitando a verificação O(n^2) de todos os pares.
  """
  import Bitwise

  # `nodes`: Sub-quadrantes (outras Quadtrees) se o nó foi dividido.
  # `level`: Profundidade do nó atual na árvore.
  # `rectangle`: As fronteiras deste nó.
  # `children`: Os objetos (partículas) contidos neste nó.
  # `max_length`: Número máximo de objetos antes de dividir o nó.
  # `max_depth`: Profundidade máxima para evitar subdivisões infinitas.
  defstruct nodes: [], level: 0, rectangle: nil, children: [], max_length: 4, max_depth: 10

  @doc """
  Cria uma nova Quadtree raiz com uma largura e altura especificadas.
  """
  def create(width: width, height: height) do
    # Supõe-se a existência de um módulo `Rectangle` para manipulação de retângulos.
    %QuadTree{rectangle: %Rectangle{width: width, height: height}}
  end

  @doc "Limpa a Quadtree, removendo todos os filhos e subnós."
  def clear(quadtree) do
    %{quadtree | nodes: []}
  end

  @doc "Divide o nó atual em quatro subnós (quadrantes)."
  defp split(quadtree) do
    # Para quem não conhece, n >>> 1 (deslocamento de bits à direita)
    # é o mesmo que floor(n / 2), mas mais rápido.
    %{rectangle: rectangle, level: level} = quadtree
    height = floor(rectangle.height) >>> 1
    width = floor(rectangle.width) >>> 1

    %{x: x, y: y} = rectangle

    # Precisamos dividir o retângulo em quatro.
    nodes =
      [
        %{x: x + width, y: y},
        %{x: x, y: y},
        %{x: x, y: y + height},
        %{x: x + width, y: y + height}
      ]
      |> Enum.map(fn %{x: x, y: y} ->
        # Constrói o retângulo a partir das coordenadas.
        %Rectangle{x: x, y: y, width: width, height: height}
      end)
      |> Enum.map(fn rect ->
        # Cria as novas Quadtrees filhas.
        %QuadTree{level: level + 1, rectangle: rect}
      end)

    new_tree = %QuadTree{quadtree | nodes: nodes}

    # Reinsere os filhos do nó original na nova estrutura dividida.
    new_tree.children
    |> Enum.reduce(%{new_tree | children: []}, fn child, tree -> insert(tree, child) end)
  end

  @doc "Encontra os subnós que um determinado retângulo pode colidir."
  defp get_node(quadtree, rectangle) do
    quadtree.nodes
    |> Enum.with_index()
    |> Enum.filter(fn {a, _} -> Rectangle.collides?(a.rectangle, rectangle) end)
  end

  @doc "Retorna todos os objetos na árvore que podem colidir com o retângulo fornecido."
  def query(quadtree, rectangle) do
    query(quadtree, rectangle, :recursive)
    |> List.flatten()
    |> Enum.filter(fn rect -> Rectangle.collides?(rect, rectangle) end)
  end

  @doc "Função auxiliar recursiva para a consulta."
  defp query(%{nodes: nodes} = quadtree, rectangle, :recursive) when length(nodes) > 0 do
    get_node(quadtree, rectangle)
    |> Enum.map(fn {a, _} -> query(a, rectangle) end)
    |> Enum.concat(quadtree.children)
  end

  defp query(%{nodes: nodes} = quadtree, _, :recursive) when length(nodes) == 0 do
    quadtree.children
  end

  @doc "Insere um objeto (retângulo) na Quadtree."
  def insert(quadtree, rectangle) do
    new_tree = insert_object(quadtree, rectangle)

    # Se o nó está cheio e não atingiu a profundidade máxima, divide-o.
    if should_expand?(new_tree) do
      split(new_tree)
    else
      new_tree
    end
  end

  @doc "Verifica se o nó tem subnós."
  defp has_subnodes?(%{nodes: nodes}) do
    length(nodes) > 0
  end

  @doc "Verifica se o nó deve ser expandido (dividido)."
  defp should_expand?(%{
         children: children,
         level: level,
         max_length: max_length,
         max_depth: max_depth
       }) do
    length(children) > max_length and level < max_depth
  end

  @doc "Lógica principal para inserir um objeto na árvore."
  defp insert_object(quadtree, rectangle) do
    if has_subnodes?(quadtree) do
      insert_object(quadtree, rectangle, :subnodes)
    else
      insert_object(quadtree, rectangle, :empty)
    end
  end

  @doc "Insere um objeto em um nó que já possui subnós."
  defp insert_object(quadtree, rectangle, :subnodes) do
    nodes = get_node(quadtree, rectangle)

    # Se o objeto cabe inteiramente em um subnó, insere-o recursivamente.
    if length(nodes) == 1 do
      [{node, i}] = nodes

      # Substitui o nó antigo pelo nó atualizado com o novo objeto.
      new_nodes =
        quadtree.nodes
        |> List.replace_at(i, insert(node, rectangle))

      %QuadTree{quadtree | nodes: new_nodes}
    else
      # Se o objeto se sobrepõe a múltiplos subnós, ele pertence ao nó pai.
      %QuadTree{quadtree | children: quadtree.children ++ [rectangle]}
    end
  end

  @doc "Insere um objeto em um nó folha (sem subnós)."
  defp insert_object(quadtree, rectangle, :empty) do
    %QuadTree{quadtree | children: quadtree.children ++ [rectangle]}
  end
end

defmodule CollisionSimulator.MixProject do
  use Mix.Project

  def project do
    [
      app: :collision_simulator,
      version: "0.1.0",
      elixir: "~> 1.14",
      elixirc_paths: elixirc_paths(Mix.env()),
      start_permanent: Mix.env() == :prod,
      aliases: aliases(),
      deps: deps()
    ]
  end

  # Configuração para a aplicação OTP.
  # Define o módulo principal da aplicação e quaisquer outras
  # aplicações OTP das quais ela depende.
  def application do
    [
      mod: {CollisionSimulator.Application, []},
      extra_applications: [:logger, :runtime_tools, :observer, :wx]
    ]
  end

  # Especifica quais caminhos compilar por ambiente.
  defp elixirc_paths(:test), do: ["lib", "test/support"]
  defp elixirc_paths(_), do: ["lib"]

  # Define as dependências do projeto.
  defp deps do
    [
      # --- Dependências Web (Phoenix) ---
      # Framework web principal.
      {:phoenix, "~> 1.7.19"},
      # Funções de ajuda para HTML.
      {:phoenix_html, "~> 4.1"},
      # Recarrega o navegador em desenvolvimento.
      {:phoenix_live_reload, "~> 1.2", only: :dev},
      # Para a interface reativa da simulação.
      {:phoenix_live_view, "~> 1.0.0"},
      # Parser de HTML para testes.
      {:floki, ">= 0.30.0", only: :test},
      # Painel de métricas e informações em tempo real.
      {:phoenix_live_dashboard, "~> 0.8.3"},
      # Construtor de assets (JavaScript).
      {:esbuild, "~> 0.8", runtime: Mix.env() == :dev},
      # Construtor de assets (CSS).
      {:tailwind, "~> 0.2", runtime: Mix.env() == :dev},
      # Conjunto de ícones SVG.
      {:heroicons,
       github: "tailwindlabs/heroicons",
       tag: "v2.1.1",
       sparse: "optimized",
       app: false,
       compile: false,
       depth: 1},

      # --- Dependências Gerais e Utilitários ---
      # Biblioteca para envio de e-mails.
      {:swoosh, "~> 1.5"},
      # Cliente HTTP.
      {:finch, "~> 0.13"},
      # Para coletar métricas da aplicação.
      {:telemetry_metrics, "~> 1.0"},
      # Para pollar métricas periodicamente.
      {:telemetry_poller, "~> 1.0"},
      # Para internacionalização.
      {:gettext, "~> 0.26"},
      # Parser de JSON.
      {:jason, "~> 1.2"},
      # Para clustering em ambientes distribuídos.
      {:dns_cluster, "~> 0.1.1"},
      # Servidor web de alta performance para Phoenix.
      {:bandit, "~> 1.5"},

      # --- Dependências de Computação Numérica e Estruturas de Dados ---
      # A biblioteca de computação numérica para Elixir.
      {:nx, "~> 0.9.2"},
      # Backend compilador para Nx (usa o XLA do Google para JIT em CPU/GPU).
      {:exla, ">= 0.9.2"},
      # Biblioteca de Quadtree, usada para otimizar colisões.
      {:quadtree, "~> 0.1.0"}
    ]
  end

  # Aliases são atalhos para tarefas comuns do projeto.
  defp aliases do
    [
      # Agrupa tarefas comuns de configuração inicial.
      setup: ["deps.get", "assets.setup", "assets.build"],
      # Instala as ferramentas de frontend.
      "assets.setup": ["tailwind.install --if-missing", "esbuild.install --if-missing"],
      # Compila os assets de frontend para desenvolvimento.
      "assets.build": ["tailwind collision_simulator", "esbuild collision_simulator"],
      # Prepara os assets para produção (minificação, etc.).
      "assets.deploy": [
        "tailwind collision_simulator --minify",
        "esbuild collision_simulator --minify",
        "phx.digest"
      ]
    ]
  end
end
