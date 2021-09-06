defmodule ViaUtils.Format do
  @moduledoc """
  Documentation for `UtilsFormat`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> UtilsFormat.hello()
      :world

  """
# Erlang float_to_binary shorthand
  @spec eftb(float(), integer()) :: binary()
  def eftb(number, num_decimals) do
    :erlang.float_to_binary(number/1, [decimals: num_decimals])
  end

  @spec eftb_deg(float(), integer()) ::binary()
  def eftb_deg(number, num_decimals) do
    :erlang.float_to_binary(ViaUtils.Math.rad2deg(number), [decimals: num_decimals])
  end

  @spec eftb_deg_sign(float(), integer()) :: binary()
  def eftb_deg_sign(number, num_decimals) do
    str = eftb_deg(number, num_decimals)
    if (number >= 0), do: "+" <> str, else: str
  end

  @spec eftb_rad(float(), integer()) ::binary()
  def eftb_rad(number, num_decimals) do
    :erlang.float_to_binary(ViaUtils.Math.deg2rad(number), [decimals: num_decimals])
  end

  @spec eftb_list(list(), integer(), binary()) :: binary()
  def eftb_list(numbers, num_decimals, separator \\ "/") do
    Enum.reduce(numbers, "", fn (number, acc) ->
      acc <> :erlang.float_to_binary(number/1, [decimals: num_decimals]) <> separator
    end)
  end

  @spec eftb_map(map(), integer(), binary()) ::binary()
  def eftb_map(keys_values, num_decimals, separator \\ ",") do
    Enum.reduce(keys_values, "", fn ({key,value}, acc) ->
      acc <> "#{inspect(key)}: " <> :erlang.float_to_binary(value/1, [decimals: num_decimals]) <> separator
    end)
  end

  @spec eftb_map_deg(map(), integer(), binary()) ::binary()
  def eftb_map_deg(keys_values, num_decimals, separator \\ ",") do
    Enum.reduce(keys_values, "", fn ({key,value}, acc) ->
      acc <> "#{inspect(key)}: " <> :erlang.float_to_binary(ViaUtils.Math.rad2deg(value), [decimals: num_decimals]) <> separator
    end)
  end

  @spec map_rad2deg(map()) :: map()
  def map_rad2deg(values) do
    Enum.reduce(values, %{}, fn ({key, value}, acc) ->
    Map.put(acc, key, ViaUtils.Math.rad2deg(value))
    end)
  end

  @spec map_deg2rad(map()) :: map()
  def map_deg2rad(values) do
    Enum.reduce(values, %{}, fn ({key, value}, acc) ->
    Map.put(acc, key, ViaUtils.Math.deg2rad(value))
    end)
  end
end
