defmodule CollisionSimulator.Rectangle do
  @moduledoc """
  Módulo auxiliar para manipulação e verificação de colisões de retângulos.
  """
  defstruct x: 0, y: 0, width: 0, height: 0

  @type t :: %__MODULE__{
          x: number(),
          y: number(),
          width: number(),
          height: number()
        }

  @doc """
  Verifica se dois retângulos colidem.
  """
  @spec collides?(t(), t()) :: boolean()
  def collides?(rect_a, rect_b) do
    # A colisão não ocorre se um retângulo está totalmente à esquerda, direita, acima ou abaixo do outro.
    not (rect_a.x + rect_a.width < rect_b.x or
           rect_a.x > rect_b.x + rect_b.width or
           rect_a.y + rect_a.height < rect_b.y or
           rect_a.y > rect_b.y + rect_b.height)
  end
end

defmodule CollisionSimulator.Quadtree do
  @moduledoc """
  Uma implementação de Quadtree otimizada em Elixir.

  Principais otimizações:
  1.  **Uso de Tuplas para Subnós**: Em vez de uma lista, os 4 subnós (quadrantes) são
      armazenados em uma tupla (`{nw, ne, sw, se}`). Isso permite acesso e atualização
      em tempo constante O(1) com `elem/2` e `put_elem/3`, em vez da complexidade O(n) de
      `List.replace_at/3`.

  2.  **Cálculo de Índice Direto**: A função `get_index/2` calcula matematicamente
      em qual quadrante um objeto se encaixa, sem a necessidade de iterar e verificar
      colisões com cada um dos quatro quadrantes, uma melhoria significativa de
      performance na inserção.

  3.  **Pré-Anexação em Listas**: Ao adicionar objetos à lista de `children` de um nó,
      usamos `[objeto | lista]` (pré-anexação em O(1)) em vez de `lista ++ [objeto]`
      (concatenação em O(n)).

  4.  **Query Tail-Recursive**: A função de busca (`query`) foi reestruturada para ser
      recursiva em cauda (`tail-recursive`), evitando o estouro da pilha de chamadas
      em árvores muito profundas e melhorando a eficiência de memória.
  """
  alias CollisionSimulator.Rectangle

  # Constantes da Quadtree definidas como atributos do módulo.
  @max_children 4
  @max_depth 10

  # `nodes` é `nil` se for uma folha, ou uma tupla de 4 Quadtrees se for um nó interno.
  # `boundary` é o retângulo que define os limites deste nó.
  # `children` são os objetos que pertencem a este nó (porque se sobrepõem a múltiplos subnós).
  defstruct boundary: nil,
            children: [],
            nodes: nil,
            level: 0

  @type t :: %__MODULE__{}

  @doc """
  Cria uma nova Quadtree com um retângulo delimitador inicial.
  """
  @spec create(Keyword.t()) :: t()
  def create(width: width, height: height) do
    %__MODULE__{
      boundary: %Rectangle{x: 0, y: 0, width: width, height: height},
      level: 0,
      children: [],
      nodes: nil
    }
  end

  @doc """
  Limpa a Quadtree, removendo todos os objetos, mas mantendo a estrutura.
  """
  @spec clear(t()) :: t()
  def clear(quadtree) do
    # Se não houver subnós, apenas limpa a lista de filhos.
    if is_nil(quadtree.nodes) do
      %{quadtree | children: []}
    else
      # Se houver subnós, limpa recursivamente cada um deles.
      {n0, n1, n2, n3} = quadtree.nodes

      new_nodes =
        {clear(n0), clear(n1), clear(n2), clear(n3)}

      %{quadtree | children: [], nodes: new_nodes}
    end
  end

  @doc """
  Insere um objeto (retângulo) na Quadtree.
  """
  @spec insert(t(), Rectangle.t()) :: t()
  def insert(quadtree, rect) do
    # Caso 1: Se a Quadtree já está dividida em subnós.
    if is_tuple(quadtree.nodes) do
      index = get_index(quadtree.boundary, rect)

      # Se o objeto cabe inteiramente em um subnó, insere-o recursivamente.
      if index != :parent do
        updated_child = insert(elem(quadtree.nodes, index), rect)
        %{quadtree | nodes: put_elem(quadtree.nodes, index, updated_child)}
      else
        # Se o objeto se sobrepõe a múltiplos subnós, ele pertence a este nó.
        %{quadtree | children: [rect | quadtree.children]}
      end
    else
      # Caso 2: Se a Quadtree é uma folha (não tem subnós).
      # Adiciona o objeto à lista de filhos deste nó.
      new_quadtree = %{quadtree | children: [rect | quadtree.children]}

      # Se o nó exceder a capacidade e a profundidade máxima não for atingida, divide-o.
      if length(new_quadtree.children) > @max_children and new_quadtree.level < @max_depth do
        split(new_quadtree)
      else
        new_quadtree
      end
    end
  end

  @doc """
  Retorna uma lista de objetos que podem colidir com o retângulo de busca.
  """
  @spec query(t(), Rectangle.t()) :: list(Rectangle.t())
  def query(quadtree, rect) do
    # A busca interna retorna candidatos. É necessário um filtro final
    # para confirmar as colisões exatas.
    do_query(quadtree, rect)
    |> Enum.filter(&Rectangle.collides?(&1, rect))
  end

  # --- Funções Privadas ---

  @doc false
  # Função de busca recursiva em cauda. `acc` acumula os resultados.
  defp do_query(quadtree, rect, acc \\ [])

  # Caso base: se for uma folha, retorna seus filhos mais o acumulador.
  defp do_query(%{nodes: nil} = quadtree, _rect, acc) do
    quadtree.children ++ acc
  end

  # Caso recursivo: se for um nó interno.
  defp do_query(%{nodes: nodes, children: my_children} = quadtree, rect, acc) do
    # Começa acumulando os filhos do nó atual.
    new_acc = my_children ++ acc

    # Determina em quais subnós a busca deve continuar.
    indices = get_intersecting_indices(quadtree.boundary, rect)

    # Reduz sobre os índices dos subnós relevantes, acumulando os resultados.
    Enum.reduce(indices, new_acc, fn index, current_acc ->
      do_query(elem(nodes, index), rect, current_acc)
    end)
  end

  @doc false
  # Divide um nó em 4 subnós e redistribui seus filhos.
  defp split(quadtree) do
    %{boundary: boundary, level: level, children: old_children} = quadtree

    sub_width = boundary.width / 2
    sub_height = boundary.height / 2
    x = boundary.x
    y = boundary.y

    # Cria os quatro novos subnós (quadrantes). Ordem: NW, NE, SW, SE
    new_nodes = {
      %__MODULE__{
        boundary: %Rectangle{x: x, y: y, width: sub_width, height: sub_height},
        level: level + 1
      },
      %__MODULE__{
        boundary: %Rectangle{x: x + sub_width, y: y, width: sub_width, height: sub_height},
        level: level + 1
      },
      %__MODULE__{
        boundary: %Rectangle{x: x, y: y + sub_height, width: sub_width, height: sub_height},
        level: level + 1
      },
      %__MODULE__{
        boundary: %Rectangle{
          x: x + sub_width,
          y: y + sub_height,
          width: sub_width,
          height: sub_height
        },
        level: level + 1
      }
    }

    # Redistribui os objetos do nó pai para os novos subnós.
    {distributed_nodes, remaining_children} =
      Enum.reduce(old_children, {new_nodes, []}, fn child_rect, {current_nodes, acc_children} ->
        index = get_index(boundary, child_rect)

        if index != :parent do
          # Insere o objeto no subnó apropriado.
          updated_node = insert(elem(current_nodes, index), child_rect)
          {put_elem(current_nodes, index, updated_node), acc_children}
        else
          # Se o objeto ainda se sobrepõe, ele permanece no nó pai.
          {current_nodes, [child_rect | acc_children]}
        end
      end)

    %{quadtree | nodes: distributed_nodes, children: remaining_children}
  end

  @doc false
  # Calcula o índice do subnó (0-3) onde um retângulo se encaixa completamente.
  # Retorna :parent se o retângulo se sobrepõe a mais de um subnó.
  defp get_index(parent_boundary, rect) do
    vertical_midpoint = parent_boundary.x + parent_boundary.width / 2
    horizontal_midpoint = parent_boundary.y + parent_boundary.height / 2

    is_top = rect.y + rect.height < horizontal_midpoint
    is_bottom = rect.y > horizontal_midpoint
    is_left = rect.x + rect.width < vertical_midpoint
    is_right = rect.x > vertical_midpoint

    cond do
      # Noroeste (NW)
      is_top and is_left -> 0
      # Nordeste (NE)
      is_top and is_right -> 1
      # Sudoeste (SW)
      is_bottom and is_left -> 2
      # Sudeste (SE)
      is_bottom and is_right -> 3
      true -> :parent
    end
  end

  @doc false
  # Retorna uma lista de índices dos subnós que se sobrepõem a um dado retângulo.
  # Essencial para a função de busca (query).
  defp get_intersecting_indices(parent_boundary, rect) do
    vm = parent_boundary.x + parent_boundary.width / 2
    hm = parent_boundary.y + parent_boundary.height / 2

    # Usa uma lista de concatenação para construir a lista de índices.
    # Esta abordagem é clara e evita a necessidade de `Enum.uniq/1`.
    []
    |> then(fn indices -> if rect.y < hm and rect.x < vm, do: [0 | indices], else: indices end)
    |> then(fn indices ->
      if rect.y < hm and rect.x + rect.width > vm, do: [1 | indices], else: indices
    end)
    |> then(fn indices ->
      if rect.y + rect.height > hm and rect.x < vm, do: [2 | indices], else: indices
    end)
    |> then(fn indices ->
      if rect.y + rect.height > hm and rect.x + rect.width > vm, do: [3 | indices], else: indices
    end)
  end
end
