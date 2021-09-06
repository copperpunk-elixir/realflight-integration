defmodule ViaUtils.Simulation do
  require Logger
  @spec publish_gps_itow_position_velocity(any(), map(), map(), any()) :: atom()
  def publish_gps_itow_position_velocity(operator_name, position_rrm, velocity_mps, group) do
    # TODO: Get correct value for itow_ms
    itow_ms = nil

    # Logger.debug(
    #   "pub gps pos/vel: #{ViaUtils.Location.to_string(position_rrm)}/#{ViaUtils.Format.eftb_map(velocity_mps, 1)}"
    # )

    ViaUtils.Comms.send_global_msg_to_group(
      operator_name,
      {group, itow_ms, position_rrm, velocity_mps},
      self()
    )
  end

  @spec publish_gps_relheading(any(), float(), any()) :: atom()
  def publish_gps_relheading(operator_name,rel_heading_rad, group) do
    # TODO: Get correct value for itow_ms
    itow_ms = nil

    # Logger.debug("pub relhdg: #{ViaUtils.Format.eftb_deg(rel_heading_rad, 1)}")

    ViaUtils.Comms.send_global_msg_to_group(
      operator_name,
      {group, itow_ms, rel_heading_rad},
      self()
    )
  end

  @spec publish_dt_accel_gyro(any(), float(), map(), map(), any()) :: atom()
  def publish_dt_accel_gyro(operator_name, dt_s, accel_mpss, gyro_rps, group) do
    values =
      %{dt_s: dt_s}
      |> Map.merge(accel_mpss)
      |> Map.merge(gyro_rps)

    # Logger.debug("pub dtaccgy: #{ViaUtils.Format.eftb_map(values,4)}")
    ViaUtils.Comms.send_global_msg_to_group(
      operator_name,
      {group, values},
      self()
    )
  end

  @spec publish_airspeed(any(), float(), any()) :: atom()
  def publish_airspeed(operator_name, airspeed_mps, group) do
    # Logger.debug("pub A/S: #{ViaUtils.Format.eftb(airspeed_mps, 1)}")

    ViaUtils.Comms.send_global_msg_to_group(
      operator_name,
      {group, airspeed_mps},
      self()
    )
  end

  @spec publish_downward_tof_distance(any(), map(), float(), any()) :: atom()
  def publish_downward_tof_distance(operator_name, attitude_rad, agl_m, group) do
    range_meas = agl_m / (:math.cos(attitude_rad.roll_rad) * :math.cos(attitude_rad.pitch_rad))
    range_meas = if range_meas < 0, do: 0, else: range_meas

    # Logger.debug("pub tof: #{ViaUtils.Format.eftb(range_meas, 1)}")

    ViaUtils.Comms.send_global_msg_to_group(
      operator_name,
      {group, range_meas},
      self()
    )
  end
end
