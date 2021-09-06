defmodule RealflightIntegration do
  alias RealflightIntegration.Utils

  @moduledoc """
  Documentation for `RealflightIntegration`.
  """
  require Logger
  use Bitwise
  use GenServer
  require ViaUtils.Comms.Groups, as: Groups

  # @rad2deg 57.295779513
  @default_latitude 41.769201
  @default_longitude -122.506394
  @default_servo [0.5, 0.5, 0, 0.5, 0.5, 0, 0.5, 0, 0.5, 0.5, 0.5, 0.5]
  @rf_stick_mult 1.07

  @exchange_data_loop :exchange_data_loop
  @dt_accel_gyro_loop :dt_accel_gyro_loop
  @gps_pos_vel_loop :gps_pos_vel_loop
  @gps_relhdg_loop :gps_relhdg_loop
  @airspeed_loop :airspeed_loop
  @down_tof_loop :down_tof_loop
  @simulation_update_actuators :simulation_update_actuators

  def start_link(config) do
    Logger.debug("Start RealflightIntegration GenServer")
    ViaUtils.Process.start_link_redundant(GenServer, __MODULE__, config, __MODULE__)
  end

  @impl GenServer
  def init(config) do
    ViaUtils.Comms.start_operator(__MODULE__)
    ViaUtils.Comms.join_group(__MODULE__, @simulation_update_actuators, self())

    url = Keyword.fetch!(config, :host_ip) <> ":18083"
    Logger.debug("url: #{url}")

    publish_dt_accel_gyro_interval_ms = config[:publish_dt_accel_gyro_interval_ms]
    publish_gps_position_velocity_interval_ms = config[:publish_gps_position_velocity_interval_ms]
    publish_gps_relative_heading_interval_ms = config[:publish_gps_relative_heading_interval_ms]
    publish_airspeed_interval_ms = config[:publish_airspeed_interval_ms]
    publish_downward_tof_distance_interval_ms = config[:publish_downward_tof_distance_interval_ms]

    state = %{
      url: url,
      bodyaccel_mpss: %{},
      attitude_rad: %{},
      bodyrate_rps: %{},
      position_rrm: %{},
      velocity_mps: %{},
      position_origin_rrm:
        ViaUtils.Location.new_location_input_degrees(@default_latitude, @default_longitude),
      agl_m: nil,
      airspeed_mps: nil,
      rcin: @default_servo,
      servo_out: @default_servo,
      rc_passthrough: Keyword.get(config, :rc_passthrough, false),
      # pwm_channels: Keyword.fetch!(config, :pwm_channels),
      # reversed_channels: Keyword.fetch!(config, :reversed_channels),
      dt_accel_gyro_group: config[:dt_accel_gyro_group],
      gps_itow_position_velocity_group: config[:gps_itow_position_velocity_group],
      gps_itow_relheading_group: config[:gps_itow_relheading_group],
      airspeed_group: config[:airspeed_group],
      downward_tof_distance_group: config[:downward_tof_distance_group],
      publish_dt_accel_gyro_interval_ms: publish_dt_accel_gyro_interval_ms,
      publish_gps_position_velocity_interval_ms: publish_gps_position_velocity_interval_ms,
      publish_gps_relative_heading_interval_ms: publish_gps_relative_heading_interval_ms,
      publish_airspeed_interval_ms: publish_airspeed_interval_ms,
      publish_downward_tof_distance_interval_ms: publish_downward_tof_distance_interval_ms
    }

    restore_controller(url)

    inject_controller_interface(url)

    ViaUtils.Process.start_loop(
      self(),
      publish_dt_accel_gyro_interval_ms,
      {@dt_accel_gyro_loop, publish_dt_accel_gyro_interval_ms * 1.0e-3}
    )

    ViaUtils.Process.start_loop(
      self(),
      publish_gps_position_velocity_interval_ms,
      @gps_pos_vel_loop
    )

    ViaUtils.Process.start_loop(
      self(),
      publish_gps_relative_heading_interval_ms,
      @gps_relhdg_loop
    )

    ViaUtils.Process.start_loop(self(), publish_downward_tof_distance_interval_ms, @down_tof_loop)
    ViaUtils.Process.start_loop(self(), config[:sim_loop_interval_ms], @exchange_data_loop)
    {:ok, state}
  end

  @impl GenServer
  def terminate(reason, state) do
    Logger.error("#{__MODULE__} terminated for #{inspect(reason)}")
    state
  end

  @impl GenServer
  def handle_cast({:post, msg, params}, state) do
    Logger.debug("post: #{msg}")

    state =
      case msg do
        :reset ->
          reset_aircraft(state.url)
          state

        :restore ->
          restore_controller(state.url)
          state

        :inject ->
          inject_controller_interface(state.url)
          state

        :exchange ->
          exchange_data(state, params)
      end

    {:noreply, state}
  end

  @impl GenServer
  def handle_cast({@simulation_update_actuators, actuators_and_outputs, is_override}, state) do
    # Logger.debug("output map: #{ViaUtils.Format.eftb_map(actuators_and_outputs,3)}")
    aileron = Map.get(actuators_and_outputs, :aileron_scaled) |> get_one_sided_value()
    elevator = Map.get(actuators_and_outputs, :elevator_scaled) |> get_one_sided_value()
    elevator = if is_override, do: elevator, else: 1-elevator
    throttle = Map.get(actuators_and_outputs, :throttle_scaled)# |> get_one_sided_value()
    rudder = Map.get(actuators_and_outputs, :rudder_scaled) |> get_one_sided_value()
    flaps = Map.get(actuators_and_outputs, :flaps_scaled) #|> get_one_sided_value()
    servo_out = [aileron, elevator, throttle, rudder, 0, flaps, 0, 0, 0, 0, 0, 0]

    # Logger.debug("servo_out: #{inspect(servo_out)}")
    {:noreply, %{state | servo_out: servo_out}}
  end

  # @impl GenServer
  # def handle_cast({:pwm_input, scaled_values}, state) do
  #   # Logger.debug("pwm ch: #{inspect(pwm_channels)}")
  #   # Logger.info("scaled: #{Common.Utils.eftb_list(scaled_values, 3)}")
  #   cmds_reverse =
  #     Enum.reduce(Enum.with_index(scaled_values), [], fn {ch_value, _index}, acc ->
  #       [ch_value] ++ acc
  #     end)

  #   cmds_reverse =
  #     Enum.reduce(1..(11 - length(scaled_values)), cmds_reverse, fn _x, acc ->
  #       [0] ++ acc
  #     end)

  #   cmds =
  #     Enum.reverse(cmds_reverse)
  #     |> List.insert_at(4, 0)

  #   # Logger.debug(Common.Utils.eftb_list(cmds, 2))
  #   {:noreply, %{state | servo_out: cmds}}
  # end

  def handle_info(@exchange_data_loop, state) do
    state = exchange_data(state, state.servo_out)

    rcin =
      Enum.map(state.rcin, fn x ->
        get_two_sided_value(x)
      end)

    ViaUtils.Comms.send_global_msg_to_group(
      __MODULE__,
      {Groups.command_channels(), rcin},
      self()
    )

    {:noreply, state}
  end

  @impl GenServer
  def handle_info({@dt_accel_gyro_loop, dt_s}, state) do
    bodyaccel_mpss = state.bodyaccel_mpss
    bodyrate_rps = state.bodyrate_rps

    unless Enum.empty?(bodyaccel_mpss) or Enum.empty?(bodyrate_rps) do
      # Logger.debug("br: #{ViaUtils.Format.eftb_map_deg(bodyrate_rps, 1)}")

      ViaUtils.Simulation.publish_dt_accel_gyro(
        __MODULE__,
        dt_s,
        bodyaccel_mpss,
        bodyrate_rps,
        state.dt_accel_gyro_group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@gps_pos_vel_loop, state) do
    position_rrm = state.position_rrm
    velocity_mps = state.velocity_mps

    unless Enum.empty?(position_rrm) or Enum.empty?(velocity_mps) do
      ViaUtils.Simulation.publish_gps_itow_position_velocity(
        __MODULE__,
        position_rrm,
        velocity_mps,
        state.gps_itow_position_velocity_group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@gps_relhdg_loop, state) do
    attitude_rad = state.attitude_rad

    unless Enum.empty?(attitude_rad) do
      ViaUtils.Simulation.publish_gps_relheading(
        __MODULE__,
        attitude_rad.yaw_rad,
        state.gps_itow_relheading_group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@airspeed_loop, state) do
    airspeed_mps = state.airspeed_mps

    unless is_nil(airspeed_mps) do
      ViaUtils.Simulation.publish_airspeed(__MODULE__, airspeed_mps, state.airspeed_group)
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@down_tof_loop, state) do
    attitude_rad = state.attitude_rad
    agl_m = state.agl_m

    unless Enum.empty?(attitude_rad) or is_nil(agl_m) do
      ViaUtils.Simulation.publish_downward_tof_distance(
        __MODULE__,
        attitude_rad,
        agl_m,
        state.downward_tof_distance_group
      )
    end

    {:noreply, state}
  end

  def fix_rx(x) do
    (x - 0.5) * @rf_stick_mult + 0.5
  end

  @spec reset_aircraft(binary()) :: binary()
  def reset_aircraft(url) do
    body = "<?xml version='1.0' encoding='UTF-8'?>
    <soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
    <soap:Body>
    <ResetAircraft><a>1</a><b>2</b></ResetAircraft>
    </soap:Body>
    </soap:Envelope>"
    Logger.debug("body: #{inspect(body)}")
    Logger.debug("reset")
    response = post_poison(url, body)
    Logger.debug("reset response: #{response}")
    # Logger.debug("#{inspect(Soap.Response.parse(response.body))}")
    response
  end

  @spec restore_controller(binary()) :: binary()
  def restore_controller(url) do
    body = "<?xml version='1.0' encoding='UTF-8'?>
    <soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
    <soap:Body>
    <RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>
    </soap:Body>
    </soap:Envelope>"
    Logger.debug("restore")
    post_poison(url, body)
  end

  @spec inject_controller_interface(binary()) :: binary()
  def inject_controller_interface(url) do
    body = "<?xml version='1.0' encoding='UTF-8'?>
    <soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
    <soap:Body>
    <InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
    </soap:Body>
    </soap:Envelope>"
    post_poison(url, body)
  end

  @spec exchange_data(map(), list()) :: atom()
  def exchange_data(state, servo_output) do
    # start_time = :os.system_time(:microsecond)
    # Logger.debug("start: #{start_time}")
    body_header = "<?xml version='1.0' encoding='UTF-8'?>
    <soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
    <soap:Body>
    <ExchangeData>
    <pControlInputs>
    <m-selectedChannels>4095</m-selectedChannels>
    <m-channelValues-0to1>"
    body_footer = "</m-channelValues-0to1>
                                 </pControlInputs>
                                 </ExchangeData>
                                 </soap:Body>
                                 </soap:Envelope>"

    servo_str =
      Enum.reduce(servo_output, "", fn value, acc ->
        acc <> "<item>#{ViaUtils.Format.eftb(value, 4)}</item>"
      end)

    body = body_header <> servo_str <> body_footer
    # Logger.debug("body: #{inspect(body)}")
    response = post_poison(state.url, body)

    xml_map =
      case SAXMap.from_string(response) do
        {:ok, xml} -> xml
        _other -> %{}
      end

    return_data = get_in(xml_map, return_data_path())
    # Logger.info("#{inspect(return_data)}")
    if is_nil(return_data) do
      state
    else
      aircraft_state = Utils.extract_from_path(return_data, aircraft_state_path())
      rcin_values = Utils.extract_from_path(return_data, rcin_path())
      position = Utils.extract_position(aircraft_state, state.position_origin_rrm)
      # Logger.debug("position: #{ViaUtils.Location.to_string(position)}")
      velocity = Utils.extract_velocity(aircraft_state)
      # Logger.debug("velocity: #{ViaUtils.Format.eftb_map(velocity, 2)}")
      attitude = Utils.extract_attitude(aircraft_state)
      # Logger.debug("attitude: #{ViaUtils.Format.eftb_map_deg(attitude, 1)}")
      bodyaccel = Utils.extract_bodyaccel(aircraft_state)
      # Logger.debug("bodyaccel: #{ViaUtils.Format.eftb_map(bodyaccel, 3)}")
      bodyrate = Utils.extract_bodyrate(aircraft_state)
      # Logger.debug(ViaUtils.Format.eftb_map_deg(bodyrate, 2))
      agl = Utils.extract_agl(aircraft_state)
      # Logger.debug("agl: #{agl}")
      airspeed = Utils.extract_airspeed(aircraft_state)
      # Logger.debug("airspeed: #{airspeed}")
      rcin = Utils.extract_rcin(rcin_values)
      servo_out = if state.rc_passthrough, do: rcin, else: state.servo_out
      # Logger.debug("rcin: #{inspect(rcin)}")
      # end_time = :os.system_time(:microsecond)
      # Logger.debug("dt: #{ViaUtils.Format.eftb((end_time-start_time)*0.001,1)}")
      %{
        state
        | bodyaccel_mpss: bodyaccel,
          bodyrate_rps: bodyrate,
          attitude_rad: attitude,
          position_rrm: position,
          velocity_mps: velocity,
          agl_m: agl,
          airspeed_mps: airspeed,
          rcin: rcin,
          servo_out: servo_out
      }
    end
  end

  @spec reset_aircraft() :: atom()
  def reset_aircraft() do
    GenServer.cast(__MODULE__, {:post, :reset, nil})
  end

  @spec restore_controller() :: atom()
  def restore_controller() do
    GenServer.cast(__MODULE__, {:post, :restore, nil})
  end

  @spec inject_controller_interface() :: atom()
  def inject_controller_interface() do
    GenServer.cast(__MODULE__, {:post, :inject, nil})
  end

  @spec set_throttle(float()) :: atom()
  def set_throttle(throttle) do
    servos =
      Enum.reduce(0..11, [], fn x, acc ->
        if x == 2, do: [throttle] ++ acc, else: [0.5] ++ acc
      end)
      |> Enum.reverse()

    GenServer.cast(__MODULE__, {:post, :exchange, servos})
  end

  @spec post_poison(binary(), binary(), integer()) :: binary()
  def post_poison(url, body, timeout \\ 10) do
    case HTTPoison.post(url, body, [], timeout: timeout) do
      {:ok, response} ->
        response.body

      {:error, error} ->
        Logger.warn("HTTPoison error: #{inspect(error)}")
        ""
    end
  end

  @spec return_data_path() :: list()
  def return_data_path() do
    ["SOAP-ENV:Envelope", "SOAP-ENV:Body", "ReturnData"]
  end

  @spec aircraft_state_path() :: list()
  def aircraft_state_path() do
    ["m-aircraftState"]
  end

  @spec rcin_path() :: list()
  def rcin_path() do
    ["m-previousInputsState", "m-channelValues-0to1", "item"]
  end

  @spec get_one_sided_value(number()) :: number()
  def get_one_sided_value(two_sided_value) do
    0.5 * two_sided_value + 0.5
  end

  @spec get_two_sided_value(number()) :: number()
  def get_two_sided_value(one_sided_value) do
    2 * one_sided_value - 1
  end
end
