defmodule CollisionSimulator.Particle do
  @moduledoc """
  Representação de processo individual para partícula.

  Papel na Arquitetura Refatorada:
  - Ator responsável pelo seu próprio estado (posição, velocidade).
  - Possui um loop de movimento (`:move`) para se atualizar e colidir com paredes.
  - Recebe uma mensagem `:update_after_collision` do `CollisionEngine` para
    ajustar sua velocidade após uma colisão com outra partícula.
  - Atualiza seu próprio registro na tabela ETS.
  """
  use GenServer

  alias CollisionSimulator.Physics
  alias CollisionSimulator.CollisionEngine

  @frame_interval_ms 16
  @dt @frame_interval_ms / 1000.0

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
    # O estado do GenServer é a própria tupla da partícula
    particle_tuple =
      {
        Keyword.fetch!(opts, :id),
        Keyword.fetch!(opts, :pos),
        Keyword.fetch!(opts, :vel),
        Keyword.fetch!(opts, :radius),
        Keyword.fetch!(opts, :mass)
      }

    # Insere o estado inicial na ETS
    :ets.insert(:particle_data, particle_tuple)
    # Agenda o primeiro passo de movimento
    Process.send_after(self(), :move, @frame_interval_ms)
    {:ok, particle_tuple}
  end

  @doc """
  Loop de movimento da partícula. Executa a integração de Euler e colisão com paredes.
  """
  @impl true
  def handle_info(:move, current_state) do
    bounds = CollisionEngine.world_bounds()
    {_id, pos, vel, radius, _mass} = current_state

    # 1. Integração de Euler para calcular a nova posição
    [px, py] = pos
    [vx, vy] = vel
    new_pos_integrated = [px + vx * @dt, py + vy * @dt]

    # 2. Lida com a colisão na parede
    {final_pos, final_vel} =
      Physics.handle_wall_collision(
        %{pos: new_pos_integrated, vel: vel, radius: radius},
        bounds
      )

    # Monta o novo estado
    new_state =
      current_state
      |> put_elem(get_attr_index(:pos), final_pos)
      |> put_elem(get_attr_index(:vel), final_vel)

    # Atualiza a ETS com seu novo estado
    :ets.insert(:particle_data, new_state)

    # Agenda o próximo movimento
    Process.send_after(self(), :move, @frame_interval_ms)

    {:noreply, new_state}
  end

  @doc """
  Recebe a atualização de velocidade do CollisionEngine após uma colisão.
  """
  @impl true
  def handle_cast({:update_after_collision, new_velocity}, current_state) do
    # Apenas atualiza a velocidade com base na informação do Engine
    new_state = put_elem(current_state, get_attr_index(:vel), new_velocity)

    # Atualiza a ETS com o estado corrigido
    :ets.insert(:particle_data, new_state)

    {:noreply, new_state}
  end

  # --- Funções de Acesso a Atributos (Helpers) ---
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
