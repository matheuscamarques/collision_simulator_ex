defmodule CollisionSimulator.Particle do
  @moduledoc """
  Representação de processo individual para partícula.

  Papel na arquitetura:
  - Ator responsável apenas por estado inicial
  - Registra-se no Registry global
  - Estado posterior é gerenciado pelo CollisionEngine via ETS

  Otimização:
  - Processos não participam do loop de simulação
  - Atualizações são centralizadas no CollisionEngine
  """
  use GenServer
  require Logger

  alias CollisionSimulator.Physics
  alias CollisionSimulator.CollisionEngine

  # ms
  @update_interval 16
  # --- Tipos ---
  @type id :: non_neg_integer()
  # CORREÇÃO: O tipo vec2 agora é uma lista para alinhar com o formato do Nx.
  @type vec2 :: [float()]
  @typep particle_tuple :: {id(), vec2(), vec2(), float(), float()}

  # --- API do Cliente ---
  @doc "Inicia o GenServer da partícula e o registra."
  @spec start_link(list()) :: GenServer.on_start()
  def start_link(opts) do
    id = Keyword.fetch!(opts, :id)
    # Usa `via_tuple` para registrar o processo com um nome baseado em seu ID.
    GenServer.start_link(__MODULE__, opts, name: via_tuple(id))
  end

  @doc """
  Via Tuple para registro global:
  - Permite acesso direto a qualquer partícula por ID
  - Usa Registry com chaves únicas
  """
  @spec via_tuple(id()) :: {:via, module(), {module(), id()}}
  def via_tuple(id), do: {:via, Registry, {CollisionSimulator.ParticleRegistry, id}}

  # --- Callbacks do Servidor ---
  @impl true
  @doc "Inicializa o estado da partícula e o insere na tabela ETS."
  @spec init(list()) :: {:ok, particle_tuple()}
  def init(opts) do
    particle_tuple =
      {
        Keyword.fetch!(opts, :id),
        Keyword.fetch!(opts, :pos),
        Keyword.fetch!(opts, :vel),
        Keyword.fetch!(opts, :radius),
        Keyword.fetch!(opts, :mass)
      }

    :ets.insert(:particle_data, particle_tuple)

    # Inicia loop de atualização (OPCIONAL: Esta lógica pode ser removida
    # se o CollisionEngine for o único responsável por atualizar o estado)
    Process.send_after(self(), :update, @update_interval)

    {:ok, particle_tuple}
  end

  # --- Funções de Acesso a Atributos (Helpers) ---

  def get_attr(particle_state, attr) do
    case attr do
      :id -> elem(particle_state, 0)
      :pos -> elem(particle_state, 1)
      :vel -> elem(particle_state, 2)
      :radius -> elem(particle_state, 3)
      :mass -> elem(particle_state, 4)
      _ -> raise ArgumentError, "Atributo desconhecido: #{inspect(attr)}"
    end
  end

  # Retorna o índice 0-based para funções do Elixir como elem/put_elem
  def get_attr_index(attr) do
    case attr do
      :id -> 0
      :pos -> 1
      :vel -> 2
      :radius -> 3
      :mass -> 4
      _ -> raise ArgumentError, "Atributo desconhecido: #{inspect(attr)}"
    end
  end

  # Retorna a posição 1-based para funções do ETS como update_element
  def get_attr_ets_position(attr) do
    get_attr_index(attr) + 1
  end

  # --- Loop de Atualização do Servidor ---

  @impl true
  def handle_info(:update, current_tuple_state) do
    new_tuple_state = update_particle(current_tuple_state)

    :ets.insert(:particle_data, new_tuple_state)

    # Agenda próxima atualização
    Process.send_after(self(), :update, @update_interval)

    {:noreply, new_tuple_state}
  end

  defp update_particle(state) do
    %{dt: dt} = GenServer.call(CollisionEngine, :get_state, :infinity)
    bounds = CollisionEngine.world_bounds()

    # CORREÇÃO: Desestruturando listas em vez de tuplas.
    [x, y] = get_attr(state, :pos)
    [vx, vy] = get_attr(state, :vel)
    # CORREÇÃO: Buscando o raio corretamente da tupla de estado.
    radius = get_attr(state, :radius)

    new_x = x + vx * dt
    new_y = y + vy * dt

    # Assumindo que Physics.handle_wall_collision também foi atualizado
    # para aceitar e retornar listas para pos e vel.
    {new_pos, new_vel} =
      Physics.handle_wall_collision(
        %{pos: [new_x, new_y], vel: [vx, vy], radius: radius},
        bounds
      )

    # CORREÇÃO: Atualizando o estado com os novos valores (listas).
    state
    |> put_elem(get_attr_index(:pos), new_pos)
    |> put_elem(get_attr_index(:vel), new_vel)
  end
end
