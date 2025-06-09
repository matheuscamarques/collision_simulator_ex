defmodule CollisionSimulator.Application do
  @moduledoc """
  Módulo principal da aplicação OTP que define a árvore de supervisão.

  Responsabilidades:
  1. Define a estrutura hierárquica de supervisão
  2. Inicializa componentes críticos na ordem correta
  3. Gerencia configurações dinâmicas do endpoint

  Ordem de inicialização crítica:
  - Registry deve iniciar antes do ParticleSupervisor para registro de processos
  - CollisionEngine deve iniciar antes para criação da ETS table
  - ParticleSupervisor inicia por último para garantir dependências
  """
  use Application

  @impl true
  @spec start(any(), any()) :: {:ok, pid()}
  def start(_type, _args) do
    children = [
      CollisionSimulatorWeb.Telemetry,
      {DNSCluster,
       query: Application.get_env(:collision_simulator, :dns_cluster_query) || :ignore},
      # Correção: O Registry precisa ser iniciado como parte da árvore de supervisão.
      {Registry, keys: :unique, name: CollisionSimulator.ParticleRegistry},
      {Phoenix.PubSub, name: CollisionSimulator.PubSub},
      {Finch, name: CollisionSimulator.Finch},
      CollisionSimulatorWeb.Endpoint,
      # Ordem Corrigida: Inicia o Engine antes do ParticleSupervisor
      # para garantir que a tabela ETS exista quando as partículas forem inicializadas.
      CollisionSimulator.CollisionEngine,
      CollisionSimulator.ParticleSupervisor
    ]

    opts = [strategy: :one_for_one, name: CollisionSimulator.Supervisor]
    Supervisor.start_link(children, opts)
  end

  @impl true
  @spec config_change(any(), any(), any()) :: :ok
  def config_change(changed, _new, removed) do
    CollisionSimulatorWeb.Endpoint.config_change(changed, removed)
    :ok
  end
end
