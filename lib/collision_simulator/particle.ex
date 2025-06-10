defmodule CollisionSimulator.Particle do
  @moduledoc """
  Representação de processo individual para partícula.
  Agora é responsável por transmitir seu próprio estado para o frontend.
  """
  use GenServer

  alias CollisionSimulator.Physics
  alias CollisionSimulator.CollisionEngine

  @frame_interval_ms 16
  @dt @frame_interval_ms / 1000.0
  # NOTA: Tópico para atualizações individuais
  @simulation_topic "particle_updates"

  @type id :: non_neg_integer()
  @type vec2 :: [float()]
  @typep particle_tuple :: {id(), vec2(), vec2(), float(), float()}

  # --- API do Cliente ---
  def start_link(opts) do
    id = Keyword.fetch!(opts, :id)
    GenServer.start_link(__MODULE__, opts, name: via_tuple(id))
  end

  def via_tuple(id), do: {:via, Registry, {CollisionSimulator.ParticleRegistry, id}}

  # --- Callbacks do Servidor ---
  @impl true
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
    Process.send_after(self(), :move, @frame_interval_ms)
    {:ok, particle_tuple}
  end

  @impl true
  def handle_info(:move, current_state) do
    bounds = CollisionEngine.world_bounds()
    {_id, pos, vel, radius, _mass} = current_state

    [px, py] = pos
    [vx, vy] = vel
    new_pos_integrated = [px + vx * @dt, py + vy * @dt]

    {final_pos, final_vel} =
      Physics.handle_wall_collision(
        %{pos: new_pos_integrated, vel: vel, radius: radius},
        bounds
      )

    new_state =
      current_state
      |> put_elem(get_attr_index(:pos), final_pos)
      |> put_elem(get_attr_index(:vel), final_vel)

    :ets.insert(:particle_data, new_state)

    # NOTA: Transmite sua própria atualização para o LiveView
    broadcast_update(new_state)

    Process.send_after(self(), :move, @frame_interval_ms)
    {:noreply, new_state}
  end

  @impl true
  def handle_cast({:update_after_collision, new_velocity}, current_state) do
    new_state = put_elem(current_state, get_attr_index(:vel), new_velocity)
    :ets.insert(:particle_data, new_state)

    # NOTA: Transmite a atualização após uma colisão também.
    broadcast_update(new_state)
    {:noreply, new_state}
  end

  # --- Funções Auxiliares ---
  defp broadcast_update(particle_tuple) do
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
