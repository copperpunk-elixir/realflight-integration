defmodule ViaUtils.Enum do
  @moduledoc """
  Documentation for `UtilsEnum`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> UtilsEnum.hello()
      :world

  """
  use Bitwise

  def assert_list(value_or_list) do
    if is_list(value_or_list) do
      value_or_list
    else
      [value_or_list]
    end
  end

  def list_to_enum(input_list) do
    input_list
    |> Enum.with_index()
    |> Map.new()
  end

  def list_to_int(x_list, bytes) do
    Enum.reduce(0..(bytes - 1), 0, fn index, acc ->
      acc + (Enum.at(x_list, index) <<< (8 * index))
    end)
  end

  @spec get_key_or_value(any(), any()) :: any()
  def get_key_or_value(keys_values, id) do
    Enum.reduce(keys_values, nil, fn {key, value}, acc ->
      cond do
        key == id -> value
        value == id -> key
        true -> acc
      end
    end)
  end

  @spec index_for_embedded_value(list(), any(), any(), integer()) :: integer()
  def index_for_embedded_value(container, key, value, index \\ 0) do
    {[item], remaining} = Enum.split(container, 1)

    if Map.get(item, key, :undefined) == value do
      index
    else
      if Enum.empty?(remaining) do
        nil
      else
        index_for_embedded_value(remaining, key, value, index + 1)
      end
    end
  end
end
