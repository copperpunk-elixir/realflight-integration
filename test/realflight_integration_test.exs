defmodule RealflightIntegrationTest do
  use ExUnit.Case
  require ViaUtils.Comms.Groups, as: Groups
  doctest RealflightIntegration

  setup do
    ViaUtils.Comms.Supervisor.start_link(nil)

    config = [
      realflight_ip: "192.168.7.188",
      dt_accel_gyro_group: Groups.dt_accel_gyro_val,
      gps_itow_position_velocity_group: Groups.gps_itow_position_velocity_val(),
      gps_itow_relheading_group: Groups.gps_itow_relheading_val(),
      airspeed_group: Groups.airspeed_val(),
      downward_tof_distance_group: Groups.downward_tof_distance_val(),
      publish_dt_accel_gyro_interval_ms: 5,
      publish_gps_position_velocity_interval_ms: 200,
      publish_gps_relative_heading_interval_ms: 200,
      publish_airspeed_interval_ms: 200,
      publish_downward_tof_distance_interval_ms: 200,
      sim_loop_interval_ms: 20,
      rc_passthrough: true
    ]

    RealflightIntegration.start_link(config)
    {:ok, []}
  end

  test "add subs test" do
    Process.sleep(1_000_000)
  end
end
