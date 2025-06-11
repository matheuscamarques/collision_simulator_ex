defmodule CollisionSimulator.ParticleSupervisor do
  @moduledoc """
  Supervisor dinâmico responsável por iniciar e gerenciar o ciclo de vida dos
  processos `Particle`.

  A principal responsabilidade deste módulo é criar o conjunto inicial de partículas
  no início da aplicação. Ele gera estados iniciais aleatórios (posição e velocidade)
  para cada partícula, garantindo que elas comecem dentro dos limites do mundo.

  Utiliza a estratégia de supervisão `:one_for_one`, o que significa que se um processo
  `Particle` falhar, ele será reiniciado individualmente sem afetar as outras
  partículas, garantindo a robustez do sistema.
  """
  use Supervisor

  @doc "Inicia o supervisor de partículas."
  @spec start_link(any()) :: Supervisor.on_start()
  def start_link(init_arg) do
    Supervisor.start_link(__MODULE__, init_arg, name: __MODULE__)
  end

  @doc """
  Callback de inicialização do supervisor. Gera as especificações dos filhos (workers)
  para cada `Particle` a ser criada.
  """
  @impl true
  @spec init(any()) :: {:ok, Supervisor.sup_flags()}
  def init(_init_arg) do
    # Obtém os parâmetros globais da simulação do CollisionEngine.
    num_particles = CollisionSimulator.CollisionEngine.num_particles()
    bounds = CollisionSimulator.CollisionEngine.world_bounds()
    radius = CollisionSimulator.CollisionEngine.particle_radius()
    mass = CollisionSimulator.CollisionEngine.particle_mass()

    # Gera o estado inicial para cada partícula.
    children =
      Enum.map(0..(num_particles - 1), fn i ->
        # Gera uma posição aleatória dentro dos limites, com uma margem para o raio.
        pos = [
          random_in_range(bounds.min_x + radius, bounds.max_x - radius),
          random_in_range(bounds.min_y + radius, bounds.max_y - radius)
        ]

        # Gera uma velocidade vetorial aleatória.
        vel = [Enum.random(-150..150) * 1.0, Enum.random(-150..150) * 1.0]

        # Cada filho é uma especificação de worker para um GenServer Particle.
        # Esta especificação informa ao supervisor como iniciar e gerenciar o processo.
        %{
          id: i,
          start:
            {CollisionSimulator.Particle, :start_link,
             [[id: i, pos: pos, vel: vel, radius: radius, mass: mass]]},
          # Reinicia o processo se ele falhar.
          restart: :permanent,
          type: :worker
        }
      end)

    # Inicia o supervisor com a lista de filhos e a estratégia :one_for_one.
    Supervisor.init(children, strategy: :one_for_one)
  end

  @doc "Gera um número de ponto flutuante aleatório dentro de um intervalo."
  @spec random_in_range(number(), number()) :: float()
  defp random_in_range(min, max), do: :rand.uniform() * (max - min) + min
end
