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
    this.particleData = { positions: [], radii: [] };

    this.handleEvent("particle_update", (payload) => {
      // payload contém {positions: [[x, y], ...], radii: [r, ...]}
      this.particleData = payload;
      window.requestAnimationFrame(() => this.drawFrame());
    });

    this.drawFrame(); // Desenha o estado inicial (provavelmente vazio)
  },

  drawFrame() {
  if (!this.ctx) return;

  const { ctx, canvas } = this;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const { positions, radii } = this.particleData;
  const len = positions.length;

  for (let i = 0; i < len; i++) {
    const [x, y] = positions[i];
    const r = radii[i];

    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(59, 130, 246, 0.8)";
    ctx.fill();
    ctx.strokeStyle = "rgba(30, 64, 175, 1)";
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.closePath();
  }
},
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
