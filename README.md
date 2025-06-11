# CollisionSimulator

Relatório detalhado em PDF: [Relatório Detalhado: Arquitetura Orientada a Atores em Elixir para Simulação de 
Colisões Elásticas Concorrentes](/Relatório%20Detalhado_%20Arquitetura%20Orientada%20a%20Atores%20em%20Elixir%20para%20Simulação%20de%20Colisões%20Elásticas%20Concorrentes.pdf)

[![Assista ao vídeo no YouTube](https://img.youtube.com/vi/BizYeUibc7A/hqdefault.jpg)](https://youtube.com/shorts/BizYeUibc7A?si=HdH5SU1-k1whUgxX)


To start your Phoenix server:

  * Run `mix setup` to install and setup dependencies
  * Start Phoenix endpoint with `mix phx.server` or inside IEx with `iex -S mix phx.server`

Now you can visit [`localhost:4000`](http://localhost:4000) from your browser.

Ready to run in production? Please [check our deployment guides](https://hexdocs.pm/phoenix/deployment.html).

## Learn more

  * Official website: https://www.phoenixframework.org/
  * Guides: https://hexdocs.pm/phoenix/overview.html
  * Docs: https://hexdocs.pm/phoenix
  * Forum: https://elixirforum.com/c/phoenix-forum
  * Source: https://github.com/phoenixframework/phoenix


Aqui está um resumo técnico conciso para o README do GitHub:

---

### 🚀 Simulador de Colisão de Partículas em Elixir/Nx

**Arquitetura Híbrida OTP + Computação Vetorizada**  
Simulador de física de partículas otimizado que combina:
- Concorrência massiva com OTP (Supervisores, GenServers, Registry)
- Processamento vetorizado via Nx/EXLA (CPU/GPU)
- Quadtree para detecção eficiente de colisões

**Principais Recursos Técnicos:**
- ⚡ **Pipeline de Física em Lote**: Atualização de 50+ partículas por frame com operações vetoriais
- 🌳 **Otimização Espacial**: Quadtree dinâmica reduz complexidade de colisões (O(n²) → O(n log n))
- 🔄 **Sincronização Eficiente**: Estado compartilhado via ETS (Erlang Term Storage)
- 📡 **Tempo Real**: Broadcast de atualizações via Phoenix PubSub/WebSockets
- ⚖️ **Controle de Frames Adaptativo**: Execução estável em ~60 FPS

**Tecnologias-Chave:**
- Elixir 1.14+ (OTP 26)
- Nx/EXLA para computação numérica
- Phoenix Framework
- Quadtree personalizada

**Fluxo de Simulação:**
```mermaid
graph LR
    A[Coleta de Estados ETS] --> B[Conversão para Tensores Nx]
    B --> C[Atualização de Movimento]
    C --> D[Construção da Quadtree]
    D --> E[Detecção de Colisões]
    E --> F[Resolução de Colisões]
    F --> G[Atualização ETS]
    G --> H[Broadcast via PubSub]
```

**Desempenho:**
- Operações totalmente vetorizadas com Nx
- Paralelismo automático via EXLA
- Gerenciamento de memória zero-copy entre processos

**Ideal para:**  
Estudos de simulações físicas, otimização de sistemas concorrentes e aplicações de alta performance com Elixir.# collision_simulator_ex
