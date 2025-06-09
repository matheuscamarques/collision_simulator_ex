defmodule CollisionSimulator.ParticleSupervisor do
  @moduledoc """
  Supervisor dinâmico para gerenciar processos de partículas.

  Características:
  - Inicializa todas as partículas com estados aleatórios
  - Estratégia one_for_one: Falhas isoladas não afetam outras
  - Coordena com CollisionEngine para parâmetros globais

  Inicialização:
  - Gera posições/velocidades iniciais dentro dos limites do mundo
  - Distribuição uniforme evitando sobreposição inicial
  """
  use Supervisor

  @spec start_link(any()) :: Supervisor.on_start()
  def start_link(init_arg) do
    Supervisor.start_link(__MODULE__, init_arg, name: __MODULE__)
  end

  @doc """
  Gera estado inicial para partículas com restrições:
  - Posição: Dentro dos limites do mundo (+ margem do raio)
  - Velocidade: Vetor aleatório com magnitude controlada
  """
  @impl true
  @spec init(any()) :: {:ok, Supervisor.sup_flags()}
  def init(_init_arg) do
    num_particles = CollisionSimulator.CollisionEngine.num_particles()
    bounds = CollisionSimulator.CollisionEngine.world_bounds()
    radius = CollisionSimulator.CollisionEngine.particle_radius()
    mass = CollisionSimulator.CollisionEngine.particle_mass()

    # Gera o estado inicial para cada partícula.
    children =
      Enum.map(0..(num_particles - 1), fn i ->
        pos = [
          random_in_range(bounds.min_x + radius, bounds.max_x - radius),
          random_in_range(bounds.min_y + radius, bounds.max_y - radius)
        ]

        vel = [Enum.random(-50..50) * 1.0, Enum.random(-50..50) * 1.0]

        # Cada filho é uma especificação de worker para um GenServer Particle.
        %{
          id: i,
          start:
            {CollisionSimulator.Particle, :start_link,
             [[id: i, pos: pos, vel: vel, radius: radius, mass: mass]]},
          restart: :permanent,
          type: :worker
        }
      end)

    Supervisor.init(children, strategy: :one_for_one)
  end

  @spec random_in_range(number(), number()) :: float()
  defp random_in_range(min, max), do: :rand.uniform() * (max - min) + min
end
