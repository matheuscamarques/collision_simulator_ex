defmodule CollisionSimulator.Application do
  @moduledoc """
  Módulo principal da aplicação OTP que define a árvore de supervisão.
  """
  use Application

  @impl true
  def start(_type, _args) do
    children = [
      CollisionSimulatorWeb.Telemetry,
      {DNSCluster,
       query: Application.get_env(:collision_simulator, :dns_cluster_query) || :ignore},
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
  def config_change(changed, _new, removed) do
    CollisionSimulatorWeb.Endpoint.config_change(changed, removed)
    :ok
  end
end
