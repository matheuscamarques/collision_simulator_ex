defmodule CollisionSimulator.SpatialHash do
  @moduledoc """
  Spatial Hashing simples para simulação de colisões 2D.
  """

  defstruct cell_size: 100, grid: %{}

  @type t :: %__MODULE__{
          cell_size: pos_integer(),
          grid: %{optional({integer(), integer()}) => MapSet.t(any())}
        }

  @doc """
  Cria uma nova instância do Spatial Hash.
  """
  def new(cell_size) do
    # Garante que cell_size seja um float para as divisões, mas o tipo na struct continua pos_integer
    %__MODULE__{cell_size: cell_size}
  end

  @doc """
  Insere um objeto identificado por `id` na posição `{x, y}`.
  """
  def insert(%__MODULE__{cell_size: size, grid: grid} = hash, {x, y}, id) do
    # MUDANÇA: Usando trunc após a divisão de floats
    cell_x = trunc(x / size)
    cell_y = trunc(y / size)
    key = {cell_x, cell_y}

    updated_grid =
      Map.update(grid, key, MapSet.new([id]), fn set ->
        MapSet.put(set, id)
      end)

    %__MODULE__{hash | grid: updated_grid}
  end

  @doc """
  Retorna os objetos próximos de `{x, y}` dentro do `radius`.
  """
  def query(%__MODULE__{cell_size: size, grid: grid}, {x, y}, radius) do
    # MUDANÇA: Usando trunc após a divisão de floats
    min_x = trunc((x - radius) / size)
    max_x = trunc((x + radius) / size)
    min_y = trunc((y - radius) / size)
    max_y = trunc((y + radius) / size)

    min_x..max_x
    |> Enum.flat_map(fn cx ->
      Enum.flat_map(min_y..max_y, fn cy ->
        Map.get(grid, {cx, cy}, MapSet.new()) |> MapSet.to_list()
      end)
    end)
  end
end
