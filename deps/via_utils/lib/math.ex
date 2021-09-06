defmodule ViaUtils.Math do
  @moduledoc """
  Documentation for `UtilsMath`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> UtilsMath.hello()
      :world

  """
  require Bitwise
  require Logger

  @spec sign(number) :: integer()
  def sign(x) do
    if x >= 0, do: 1, else: -1
  end

  @spec constrain(number(), number(), number()) :: number()
  def constrain(x, min_value, max_value) do
    case x do
      _ when x > max_value -> max_value
      _ when x < min_value -> min_value
      x -> x
    end
  end

  @spec in_range?(number(), number(), number()) :: boolean()
  def in_range?(x, min_value, max_value) do
    cond do
      x > max_value -> false
      x < min_value -> false
      true -> true
    end
  end

  @spec constrain?(number(), number(), number()) :: tuple()
  def constrain?(x, min_value, max_value) do
    case x do
      _ when x > max_value -> {max_value, true}
      _ when x < min_value -> {min_value, true}
      x -> {x, false}
    end
  end

  @spec map_value(number(), number(), number(), number(), number()) :: number()
  def map_value(original_value, from_min_value, from_max_value, to_min_value, to_max_value) do
    (original_value - from_min_value)*(to_max_value - to_min_value)/(from_max_value - from_min_value) + to_min_value
  end

  @spec apply_deadband(number(), number()) :: number()
  def apply_deadband(value, deadband) do
    if (value > deadband) or (value < -deadband), do: value, else: 0
  end

  @spec hypot(number(), number()) :: float()
  def hypot(x, y) do
    :math.sqrt(x * x + y * y)
  end

  @spec hypot(tuple()) :: float()
  def hypot({x, y}) do
    :math.sqrt(x * x + y * y)
  end

  @spec hypot3(number(), number(), number()) :: float()
  def hypot3(x, y, z) do
    :math.sqrt(x * x + y * y + z * z)
  end

  def cross_product(v1, v2) do
    elem(v1, 0) * elem(v2, 1) - elem(v1, 1) * elem(v2, 0)
  end

  @spec rad2deg(number()) :: float()
  def rad2deg(x) do
    x * 180 / :math.pi()
  end

  @spec deg2rad(number()) :: float()
  def deg2rad(x) do
    x * :math.pi() / 180
  end

  @spec rotate_point(float(), float(), float()) :: tuple()
  def rotate_point(dx, dy, theta_rotate) do
    gamma = :math.atan2(dy, dx)
    hypot = hypot(dx, dy)
    x = hypot * :math.cos(gamma + theta_rotate)
    y = hypot * :math.sin(gamma + theta_rotate)
    {x, y}
  end

  @spec constrain_angle_to_compass(number()) :: number()
  def constrain_angle_to_compass(angle) do
    cond do
      angle < 0.0 -> angle + 2.0 * :math.pi()
      angle >= 2.0 * :math.pi() -> angle - 2.0 * :math.pi()
      true -> angle
    end
  end

  def integer_power(x, pow) do
    Enum.reduce(1..pow, 1, fn _iter, acc ->
      x * acc
    end)
  end

  def fp_from_uint(x, bits) do
    {sig_start, exp_min_index, exp_subtract, significand_div, exp_and} =
      case bits do
        32 -> {0x7FFFFF, 23, 127, 8_388_608, 0x100}
        64 -> {0xFFFFFFFFFFFFF, 52, 1023, 0x10000000000000, 0x800}
      end

    significand = Bitwise.&&&(sig_start, x)
    exponent = Bitwise.>>>(x, exp_min_index)
    exponent = exponent - Bitwise.&&&(exponent, exp_and)
    exponent = exponent - exp_subtract

    sign =
      if Bitwise.>>>(x, bits - 1) == 1 do
        -1
      else
        1
      end

    exp_mult =
      if exponent > 0 do
        Bitwise.<<<(1, exponent)
      else
        1 / Bitwise.<<<(1, -exponent)
      end

    sign * (1 + significand / significand_div) * exp_mult
  end

  @spec uint_from_fp(number(), integer) :: binary()
  def uint_from_fp(x, bits) do
    {exponent_add, max_value, default_value, exp_min_index} =
      case bits do
        32 -> {127, 3.4e38, <<0, 0, 0, 0>>, 23}
        64 -> {1023, 1.0e300, <<0, 0, 0, 0, 0, 0, 0, 0>>, 52}
      end

    x = x + 1 - 1

    if x == 0 or x > max_value or x < -max_value do
      # Logger.debug("use default")
      default_value
    else
      abs_x = abs(x)
      # dividing by log2
      exponent = floor(:math.log(abs_x) / 0.69314718056)
      biased_exponent = exponent + exponent_add
      exponent_bin = :erlang.integer_to_binary(biased_exponent, 2)
      # add leading zeros if necessary
      num_zeros = 8 - String.length(exponent_bin)

      exponent_bin =
        if num_zeros > 0 do
          Enum.reduce(1..num_zeros, exponent_bin, fn _x, acc ->
            "0" <> acc
          end)
        else
          exponent_bin
        end

      exp_mult = :math.pow(2, exponent)

      {_mantissa, mantissa_string} =
        Enum.reduce(1..exp_min_index, {1, ""}, fn ctr, {mantissa, mantissa_string} ->
          mantissa_temp = mantissa + 1.0 / Bitwise.<<<(1, ctr)

          if mantissa_temp * exp_mult <= abs_x do
            {mantissa_temp, mantissa_string <> "1"}
          else
            {mantissa, mantissa_string <> "0"}
          end
        end)

      number =
        if x >= 0 do
          "0"
        else
          "1"
        end

      number = number <> exponent_bin <> mantissa_string
      num_int = :erlang.binary_to_integer(number, 2)

      case bits do
        32 -> <<num_int::little-unsigned-32>>
        64 -> <<num_int::little-unsigned-64>>
      end
    end
  end

  def twos_comp_8(x) do
    <<si::signed-integer-8>> = <<x::unsigned-integer-8>>
    si
  end

  def twos_comp_8_bin(x) do
    <<si::signed-integer-8>> = x
    si
  end

  def twos_comp_16(x) do
    <<si::signed-integer-16>> = <<x::unsigned-integer-16>>
    si
  end

  def twos_comp_16_bin(x) do
    <<si::signed-integer-16>> = x
    si
  end

  def twos_comp_32(x) do
    <<si::signed-integer-32>> = <<x::unsigned-integer-32>>
    si
  end

  def twos_comp_32_bin(x) do
    <<si::signed-integer-32>> = x
    si
  end

  def twos_comp_64(x) do
    <<si::signed-integer-64>> = <<x::unsigned-integer-64>>
    si
  end

  def twos_comp_64_bin(x) do
    <<si::signed-integer-64>> = x
    si
  end

  def twos_comp(x, bits) do
    case bits do
      8 -> twos_comp_8(x)
      16 -> twos_comp_16(x)
      32 -> twos_comp_32(x)
      64 -> twos_comp_64(x)
    end
  end

  def int8_little_bin(x) do
    <<x::little-signed-integer-8>>
  end

  def int16_little_bin(x) do
    <<x::little-signed-integer-16>>
  end

  def int32_little_bin(x) do
    <<x::little-signed-integer-32>>
  end

  def int64_little_bin(x) do
    <<x::little-signed-integer-64>>
  end

  def int_little_bin(x, bits) do
    case bits do
      8 -> int8_little_bin(x)
      16 -> int16_little_bin(x)
      32 -> int32_little_bin(x)
      64 -> int64_little_bin(x)
    end
  end
end
