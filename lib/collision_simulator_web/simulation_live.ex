defmodule CollisionSimulatorWeb.SimulationLive do
  use CollisionSimulatorWeb, :live_view

  # NOTA: Tópico atualizado para corresponder ao que as partículas publicam
  @simulation_topic "particle_updates"

  @impl true
  def mount(_params, _session, socket) do
    if connected?(socket) do
      Phoenix.PubSub.subscribe(CollisionSimulator.PubSub, @simulation_topic)
    end

    # NOTA: Não precisamos mais enviar dados iniciais para o hook.
    # O canvas irá popular-se à medida que as atualizações chegam.
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

   # NOTA: O handle_info foi completamente alterado para lidar com o novo evento.
  @impl true
  def handle_info({:particle_moved, payload}, socket) do
    # Simplesmente encaminha o payload da partícula para o hook no frontend
    {:noreply, push_event(socket, "particle_moved", payload)}
  end
end
