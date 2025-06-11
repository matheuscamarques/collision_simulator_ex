// If you want to use Phoenix channels, run `mix help phx.gen.channel`
// to get started and then uncomment the line below.
// import "./user_socket.js"

// You can include dependencies in two ways.
//
// The simplest option is to put them in assets/vendor and
// import them using relative paths:
//
//     import "../vendor/some-package.js"
//
// Alternatively, you can `npm install some-package --prefix assets` and import
// them using a path starting with the package name:
//
//     import "some-package"
//

// Include phoenix_html to handle method=PUT/DELETE in forms and buttons.
import "phoenix_html";
// Establish Phoenix Socket and LiveView configuration.
import { Socket } from "phoenix";
import { LiveSocket } from "phoenix_live_view";
import topbar from "../vendor/topbar";

let Hooks = {};
Hooks.CanvasHook = {
  mounted() {
    this.canvas = this.el.querySelector("#physics-canvas");
    if (!this.canvas) {
      console.error("Elemento canvas não encontrado!");
      return;
    }

    this.ctx = this.canvas.getContext("2d");
    
    // 1. MUDANÇA: Usando um Map para armazenar as partículas por ID.
    this.particles = new Map();

    // 2. MUDANÇA: Escutando o novo evento "particle_moved".
    this.handleEvent("particle_moved", (payload) => {
      // O payload agora é de uma única partícula: {id, pos: [x, y], radius}
      const { id, pos, radius } = payload;
      
      // Atualizamos os dados da partícula no Map.
      this.particles.set(id, { x: pos[0], y: pos[1], r: radius });
    });
    
    // 4. MUDANÇA: Iniciamos um loop de animação contínuo.
    this.animationFrameId = window.requestAnimationFrame(() => this.drawFrame());
  },

  // Adicionado: Limpa o loop de animação para evitar vazamento de memória.
  destroyed() {
    if(this.animationFrameId) {
      window.cancelAnimationFrame(this.animationFrameId);
    }
  },

  drawFrame() {
  if (!this.ctx) return;

  const { ctx, canvas } = this;

  // Fundo preto
  ctx.fillStyle = "black";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Itera com índice
  const entries = Array.from(this.particles.values());
  entries.forEach((particle, index) => {
    const { x, y, r } = particle;

    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);

    // Índices pares = vermelho, ímpares = verde
    ctx.fillStyle = index % 2 === 0 ? "red" : "green";
    ctx.fill();

    ctx.strokeStyle = "white"; // Contorno branco para melhor visualização
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.closePath();
  });

  this.animationFrameId = window.requestAnimationFrame(() => this.drawFrame());
  }
};

let csrfToken = document
  .querySelector("meta[name='csrf-token']")
  .getAttribute("content");
  
let liveSocket = new LiveSocket("/live", Socket, {
  longPollFallbackMs: 2500,
  params: { _csrf_token: csrfToken },
  hooks: Hooks,
});

// Show progress bar on live navigation and form submits
topbar.config({ barColors: { 0: "#29d" }, shadowColor: "rgba(0, 0, 0, .3)" });
window.addEventListener("phx:page-loading-start", (_info) => topbar.show(300));
window.addEventListener("phx:page-loading-stop", (_info) => topbar.hide());

// connect if there are any LiveViews on the page
liveSocket.connect();

// expose liveSocket on window for web console debug logs and latency simulation:
// >> liveSocket.enableDebug()
// >> liveSocket.enableLatencySim(1000)
// >> liveSocket.disableLatencySim()
window.liveSocket = liveSocket;
