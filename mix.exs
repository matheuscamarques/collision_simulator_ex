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
