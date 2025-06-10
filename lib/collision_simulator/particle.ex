defmodule CollisionSimulator.Particle do
  @moduledoc """
  Representa uma única partícula na simulação como um processo `GenServer` individual.

  Cada partícula é responsável por:
  - Manter seu próprio estado (ID, posição, velocidade, raio, massa).
  - Atualizar sua posição em cada "tick" de movimento.
  - Detectar e resolver colisões com as paredes do mundo.
  - Manter seu estado atualizado na tabela ETS `:particle_data` para que o `CollisionEngine` possa acessá-lo.
  - Aceitar atualizações de velocidade do `CollisionEngine` após uma colisão com outra partícula.
  - **Transmitir seu próprio estado atualizado (`id`, `pos`, `radius`) para o frontend via Phoenix.PubSub.**
  """
  use GenServer

  alias CollisionSimulator.Physics
  alias CollisionSimulator.CollisionEngine

  @frame_interval_ms 16
  @dt @frame_interval_ms / 1000.0 # Delta time em segundos para cálculos de física.
  @simulation_topic "particle_updates" # Tópico do PubSub para atualizações.

  @type id :: non_neg_integer()
  @type vec2 :: [float()]
  # O estado da partícula é armazenado como uma tupla para otimização de acesso no ETS.
  @typep particle_tuple :: {id(), vec2(), vec2(), float(), float()}

  # --- API do Cliente ---

  @doc "Inicia o GenServer da partícula."
  def start_link(opts) do
    id = Keyword.fetch!(opts, :id)
    # Usa um Registry para registrar o processo com um nome baseado no ID,
    # permitindo fácil acesso através de `via_tuple`.
    GenServer.start_link(__MODULE__, opts, name: via_tuple(id))
  end

  @doc "Retorna a tupla `via` para encontrar um processo de partícula no Registry."
  def via_tuple(id), do: {:via, Registry, {CollisionSimulator.ParticleRegistry, id}}

  # --- Callbacks do Servidor ---

  @impl true
  def init(opts) do
    # Constrói a tupla de estado inicial a partir das opções.
    particle_tuple =
      {
        Keyword.fetch!(opts, :id),
        Keyword.fetch!(opts, :pos),
        Keyword.fetch!(opts, :vel),
        Keyword.fetch!(opts, :radius),
        Keyword.fetch!(opts, :mass)
      }

    # Insere seu estado inicial na tabela ETS compartilhada.
    :ets.insert(:particle_data, particle_tuple)

    # Agenda seu primeiro movimento.
    Process.send_after(self(), :move, @frame_interval_ms)
    {:ok, particle_tuple}
  end

  @impl true
  @doc """
  Manipula a mensagem de movimento. Atualiza a posição, verifica colisões com as paredes
  e transmite seu novo estado.
  """
  def handle_info(:move, current_state) do
    bounds = CollisionEngine.world_bounds()
    {_id, pos, vel, radius, _mass} = current_state

    # Integração de Euler simples para calcular a nova posição.
    [px, py] = pos
    [vx, vy] = vel
    new_pos_integrated = [px + vx * @dt, py + vy * @dt]

    # Resolve colisões com as paredes.
    {final_pos, final_vel} =
      Physics.handle_wall_collision(
        %{pos: new_pos_integrated, vel: vel, radius: radius},
        bounds
      )

    # Cria a nova tupla de estado.
    new_state =
      current_state
      |> put_elem(get_attr_index(:pos), final_pos)
      |> put_elem(get_attr_index(:vel), final_vel)

    # Atualiza seu estado na tabela ETS.
    :ets.insert(:particle_data, new_state)

    # Transmite sua própria atualização para o frontend (LiveView).
    broadcast_update(new_state)

    # Agenda o próximo movimento.
    Process.send_after(self(), :move, @frame_interval_ms)
    {:noreply, new_state}
  end

  @impl true
  @doc """
  Manipula a atualização de velocidade vinda do `CollisionEngine` após uma colisão.
  """
  def handle_cast({:update_after_collision, new_velocity}, current_state) do
    new_state = put_elem(current_state, get_attr_index(:vel), new_velocity)
    :ets.insert(:particle_data, new_state)

    # Transmite a atualização após a colisão também.
    broadcast_update(new_state)
    {:noreply, new_state}
  end

  # --- Funções Auxiliares ---

  @doc "Transmite o estado atualizado da partícula para o tópico do Phoenix PubSub."
  defp broadcast_update(particle_tuple) do
    # Monta um payload apenas com os dados necessários para a renderização no frontend.
    payload = %{
      id: elem(particle_tuple, get_attr_index(:id)),
      pos: elem(particle_tuple, get_attr_index(:pos)),
      radius: elem(particle_tuple, get_attr_index(:radius))
    }

    Phoenix.PubSub.broadcast(
      CollisionSimulator.PubSub,
      @simulation_topic,
      {:particle_moved, payload}
    )
  end

  @doc "Retorna o índice de um atributo na tupla de estado da partícula."
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
end
