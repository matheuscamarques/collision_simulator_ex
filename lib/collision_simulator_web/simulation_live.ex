defmodule CollisionSimulatorWeb.SimulationLive do
  use CollisionSimulatorWeb, :live_view

  @simulation_topic "simulation_updates"

  @impl true
  def mount(_params, _session, socket) do
    if connected?(socket) do
      Phoenix.PubSub.subscribe(CollisionSimulator.PubSub, @simulation_topic)
    end

    initial_particle_data = %{
      positions: [],
      radii: []
    }

    socket =
      assign(socket, :particle_data_for_hook, initial_particle_data)

    {:ok, socket}
  end

  @impl true
  def render(assigns) do
    ~H"""
    <div id="simulation-container" class="flex flex-col items-center p-4">
      <h1 class="text-2xl font-bold mb-4">Simulação de Colisão 2D com Nx e LiveView</h1>
      <div id="simulation-wrapper" phx-hook="CanvasHook">
        <canvas
          id="physics-canvas"
          width="800"
          height="600"
          class="border border-gray-800 bg-gray-100"
          phx-update="ignore"
        >
          Seu navegador não suporta o elemento canvas.
        </canvas>
      </div>
      Para depuração: <pre class="mt-4 text-xs"><%= inspect @particle_data_for_hook %></pre>
    </div>
    """
  end

  @impl true
  def handle_info({:particle_data, payload}, socket) do
    {:noreply, push_event(socket, "particle_update", payload)}
  end
end
