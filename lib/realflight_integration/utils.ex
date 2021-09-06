defmodule RealflightIntegration.Utils do
  require Logger
  require ViaUtils.Constants, as: VC

  @spec convert_all_to_float(map(), number()) :: map()
  def convert_all_to_float(string_values, mult \\ 1) do
    Enum.reduce(string_values, %{}, fn {key, value}, acc ->
      {x_float, _rem} = Float.parse(value)
      Map.put(acc, key, x_float * mult)
    end)
  end

  @spec extract_agl(map()) :: float()
  def extract_agl(aircraft_state) do
    agl = Map.get(aircraft_state, "m-altitudeAGL-MTR")
    if is_nil(agl), do: nil, else: String.to_float(agl)
  end

  @spec extract_airspeed(map()) :: float()
  def extract_airspeed(aircraft_state) do
    airspeed = Map.get(aircraft_state, "m-airspeed-MPS")

    if is_nil(airspeed) do
      nil
    else
      {x_float, _rem} = Float.parse(airspeed)
      x_float
    end
  end

  @spec extract_all_or_nothing(map(), list(), map()) :: map()
  def extract_all_or_nothing(data, lookup_store_tuples, accumulator \\ %{}) do
    if Enum.empty?(lookup_store_tuples) do
      accumulator
    else
      {[{lookup_key, store_key}], lookup_store_tuples} = Enum.split(lookup_store_tuples, 1)
      value = Map.get(data, lookup_key)

      if is_nil(value) do
        Logger.debug("missing value: exit")
        %{}
      else
        # Logger.debug("good value, continue: #{inspect(lookup_store_tuples)}")
        extract_all_or_nothing(data, lookup_store_tuples, Map.put(accumulator, store_key, value))
      end
    end
  end

  @spec extract_attitude(map()) :: map()
  def extract_attitude(aircraft_state) do
    quat = extract_quat(aircraft_state)
    # Logger.debug("quat: #{inspect(quat)}")
    ViaUtils.Motion.quaternion_to_euler(quat.w, quat.x, quat.y, -quat.z)
  end

  @spec extract_bodyaccel(map()) :: map()
  def extract_bodyaccel(aircraft_state) do
    lookup_keys = [
      "m-accelerationBodyAX-MPS2",
      "m-accelerationBodyAY-MPS2",
      "m-accelerationBodyAZ-MPS2"
    ]

    store_keys = [:ax_mpss, :ay_mpss, :az_mpss]
    accel_data = extract_all_or_nothing(aircraft_state, Enum.zip(lookup_keys, store_keys))

    if Enum.empty?(accel_data) do
      %{}
    else
      convert_all_to_float(accel_data)
    end
  end

  @spec extract_bodyrate(map()) :: map()
  def extract_bodyrate(aircraft_state) do
    lookup_keys = ["m-rollRate-DEGpSEC", "m-pitchRate-DEGpSEC", "m-yawRate-DEGpSEC"]
    store_keys = [:gx_rps, :gy_rps, :gz_rps]
    rate_data = extract_all_or_nothing(aircraft_state, Enum.zip(lookup_keys, store_keys))

    if Enum.empty?(rate_data) do
      %{}
    else
      bodyrate = convert_all_to_float(rate_data, VC.deg2rad())
      Map.replace!(bodyrate, :gz_rps, -bodyrate.gz_rps)
    end
  end

  @spec extract_from_path(map(), list()) :: map()
  def extract_from_path(data, path) do
    if Enum.empty?(path) do
      data
    else
      {[next_path], remaining_path} = Enum.split(path, 1)
      # Logger.debug("next: #{next_path}")
      # Logger.debug("remaining: #{inspect(remaining_path)}")
      data = Map.get(data, next_path, %{})
      extract_from_path(data, remaining_path)
    end
  end

  @spec extract_quat(map()) :: map()
  def extract_quat(aircraft_state) do
    lookup_keys = [
      "m-orientationQuaternion-W",
      "m-orientationQuaternion-X",
      "m-orientationQuaternion-Y",
      "m-orientationQuaternion-Z"
    ]

    store_keys = [:w, :y, :x, :z]
    quat_data = extract_all_or_nothing(aircraft_state, Enum.zip(lookup_keys, store_keys))

    if Enum.empty?(quat_data) do
      %{}
    else
      convert_all_to_float(quat_data)
    end
  end

  @spec extract_position(map(), struct()) :: map()
  def extract_position(aircraft_state, origin) do
    lookup_keys = ["m-aircraftPositionX-MTR", "m-aircraftPositionY-MTR", "m-altitudeASL-MTR"]
    store_keys = [:y, :x, :z]
    pos_data = extract_all_or_nothing(aircraft_state, Enum.zip(lookup_keys, store_keys))

    if Enum.empty?(pos_data) do
      %{}
    else
      # Logger.debug("#{inspect(pos_data)}")
      pos = convert_all_to_float(pos_data)

      Map.from_struct(ViaUtils.Location.location_from_point_with_dx_dy(origin, pos.x, pos.y))
      |> Map.put(:altitude_m, pos.z)
    end
  end

  @spec extract_rcin(map()) :: list()
  def extract_rcin(input_state) do
    Enum.map(input_state, fn input ->
      {input_float, _} = Float.parse(input)
      # (input_float - 0.5) * 2.0
      input_float
    end)
  end

  @spec extract_roll_inclination_azimuth(map()) :: map()
  def extract_roll_inclination_azimuth(aircraft_state) do
    lookup_keys = ["m-roll-DEG", "m-inclination-DEG", "m-azimuth-DEG"]
    store_keys = [:roll_rad, :pitch_rad, :yaw_rad]
    rate_data = extract_all_or_nothing(aircraft_state, Enum.zip(lookup_keys, store_keys))

    if Enum.empty?(rate_data) do
      %{}
    else
      convert_all_to_float(rate_data, VC.deg2rad())
    end
  end

  @spec extract_velocity(map()) :: map()
  def extract_velocity(aircraft_state) do
    lookup_keys = ["m-velocityWorldU-MPS", "m-velocityWorldV-MPS", "m-velocityWorldW-MPS"]
    store_keys = [:north_mps, :east_mps, :down_mps]
    vel_data = extract_all_or_nothing(aircraft_state, Enum.zip(lookup_keys, store_keys))

    if Enum.empty?(vel_data) do
      %{}
    else
      convert_all_to_float(vel_data)
    end
  end
end
