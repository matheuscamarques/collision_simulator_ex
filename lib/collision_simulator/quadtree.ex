defmodule CollisionSimulator.Quadtree do
  @moduledoc """
  Documentation for QuadTree.
  """
  import Bitwise

  defstruct nodes: [], level: 0, rectangle: nil, children: [], max_length: 4, max_depth: 10

  @doc """
  Hello world.
  """
  def create(width: width, height: height) do
    %QuadTree{rectangle: %Rectangle{width: width, height: height}}
  end

  def clear(quadtree) do
    %{quadtree | nodes: []}
  end

  defp split(quadtree) do
    # For those unfamiliar, n >>> 1 (bitwise right shift)
    # is the same as floor(n / 2)
    # Just faster

    %{rectangle: rectangle, level: level} = quadtree
    height = floor(rectangle.height) >>> 1
    width = floor(rectangle.width) >>> 1

    %{x: x, y: y} = rectangle

    # We need to split the rectangle into fourths
    nodes =
      [
        %{x: x + width, y: y},
        %{x: x, y: y},
        %{x: x, y: y + height},
        %{x: x + width, y: y + height}
      ]
      |> Enum.map(fn %{x: x, y: y} ->
        # Build the rectangle from the coordinates
        %Rectangle{x: x, y: y, width: width, height: height}
      end)
      |> Enum.map(fn rect ->
        %QuadTree{level: level + 1, rectangle: rect}
      end)

    new_tree = %QuadTree{quadtree | nodes: nodes}

    new_tree.children
    |> Enum.reduce(%{new_tree | children: []}, fn child, tree -> insert(tree, child) end)
  end

  defp get_node(quadtree, rectangle) do
    quadtree.nodes
    |> Enum.with_index()
    |> Enum.filter(fn {a, _} -> Rectangle.collides?(a.rectangle, rectangle) end)
  end

  def query(quadtree, rectangle) do
    query(quadtree, rectangle, :recursive)
    |> List.flatten()
    |> Enum.filter(fn rect -> Rectangle.collides?(rect, rectangle) end)
  end

  defp query(%{nodes: nodes} = quadtree, rectangle, :recursive) when length(nodes) > 0 do
    get_node(quadtree, rectangle)
    |> Enum.map(fn {a, _} -> query(a, rectangle) end)
    |> Enum.concat(quadtree.children)
  end

  defp query(%{nodes: nodes} = quadtree, _, :recursive) when length(nodes) == 0 do
    quadtree.children
  end

  def insert(quadtree, rectangle) do
    new_tree = insert_object(quadtree, rectangle)

    if should_expand?(new_tree) do
      split(new_tree)
    else
      new_tree
    end
  end

  defp has_subnodes?(%{nodes: nodes}) do
    length(nodes) > 0
  end

  defp should_expand?(%{
         children: children,
         level: level,
         max_length: max_length,
         max_depth: max_depth
       }) do
    length(children) > max_length and level < max_depth
  end

  defp insert_object(quadtree, rectangle) do
    if has_subnodes?(quadtree) do
      insert_object(quadtree, rectangle, :subnodes)
    else
      insert_object(quadtree, rectangle, :empty)
    end
  end

  defp insert_object(quadtree, rectangle, :subnodes) do
    nodes = get_node(quadtree, rectangle)

    # If our object is only in one rectangle, insert it into the correct child node
    if length(nodes) == 1 do
      [{node, i}] = nodes

      # Basically just replace the node list with an updated one
      # Recursively calls the insert function
      new_nodes =
        quadtree.nodes
        |> List.replace_at(i, insert(node, rectangle))

      %QuadTree{quadtree | nodes: new_nodes}
    else
      # If our object overlaps rectangles, it gets added to the parent
      %QuadTree{quadtree | children: quadtree.children ++ [rectangle]}
    end
  end

  defp insert_object(quadtree, rectangle, :empty) do
    %QuadTree{quadtree | children: quadtree.children ++ [rectangle]}
  end
end
