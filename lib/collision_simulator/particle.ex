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
