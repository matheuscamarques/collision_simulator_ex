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
          width="500"
          height="500"
          class="border border-gray-800 bg-gray-100 w-full max-w-[500px] aspect-square"
        />
      </div>
    </div>
    """
  end

  @impl true
  def handle_info({:particle_data, payload}, %{assigns: %{particle_data_for_hook: prev}} = socket) do
    if payload == prev do
      {:noreply, socket}
    else
      socket =
        socket
        |> assign(:particle_data_for_hook, payload)
        |> push_event("particle_update", payload)

      {:noreply, socket}
    end
  end
end
