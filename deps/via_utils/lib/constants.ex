defmodule ViaUtils.Constants do
  @moduledoc """
  Documentation for `UtilsConstants`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> UtilsConstants.hello()
      :world

  """
  defmacro deg2rad, do: 0.017453293
  defmacro earth_radius_m, do: 6_371_008.8
  defmacro ft2m, do: 0.3048
  defmacro gravity, do: 9.80665
  defmacro knots2mps, do: 0.51444444
  defmacro pi_2, do: 1.5707963
  defmacro pi_4, do: 0.785398
  defmacro rad2deg, do: 57.295779513
  defmacro two_pi, do: 6.283185
end
