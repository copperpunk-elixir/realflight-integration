defmodule RealflightIntegration.SendReceive do
  alias RealflightIntegration.Utils

  @moduledoc """
  Documentation for `RealflightIntegration`.
  """
  require Logger
  use Bitwise
  use GenServer
  require ViaUtils.Shared.Groups, as: Groups
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.File
  require ViaUtils.Ubx.ClassDefs, as: ClassDefs
  require ViaUtils.Ubx.VehicleCmds.ActuatorCmdDirect, as: ActuatorCmdDirect
  require ViaUtils.Ubx.VehicleCmds.BodyrateActuatorOutput, as: BodyrateActuatorOutput
  require ViaUtils.Shared.ActuatorNames, as: Act
  alias ViaUtils.Watchdog

  @default_latitude 41.769201
  @default_longitude -122.506394
  @default_servo [0.5, 0.5, 0, 0.5, 0.5, 0, 0.5, 0, 0.5, 0.5, 0.5, 0.5]
  @start_exchange_data_loop :start_exchange_data_loop
  @exchange_data_loop :exchange_data_loop
  @dt_accel_gyro_loop :dt_accel_gyro_loop
  @gps_pos_vel_loop :gps_pos_vel_loop
  @gps_relhdg_loop :gps_relhdg_loop
  @airspeed_loop :airspeed_loop
  @down_range_loop :down_range_loop
  @clear_exchange_callback :clear_exchange_callback

  @url_port 18083

  @ip_filename "realflight.txt"
  def start_link(config) do
    Logger.debug("Start RealflightIntegration GenServer")
    ViaUtils.Process.start_link_redundant(GenServer, __MODULE__, config, __MODULE__)
  end

  @impl GenServer
  def init(config) do
    ViaUtils.Comms.start_operator(__MODULE__)

    realflight_ip_address = Keyword.get(config, :realflight_ip)

    publish_dt_accel_gyro_interval_ms = config[:publish_dt_accel_gyro_interval_ms]
    publish_gps_position_velocity_interval_ms = config[:publish_gps_position_velocity_interval_ms]
    publish_gps_relative_heading_interval_ms = config[:publish_gps_relative_heading_interval_ms]
    publish_airspeed_interval_ms = config[:publish_airspeed_interval_ms]

    publish_downward_range_distance_interval_ms =
      config[:publish_downward_range_distance_interval_ms]

    state = %{
      realflight_ip_address: realflight_ip_address,
      host_ip_address: nil,
      url: nil,
      ubx: UbxInterpreter.new(),
      bodyaccel_mpss: %{},
      attitude_rad: %{},
      bodyrate_rps: %{},
      position_rrm: %{},
      velocity_mps: %{},
      position_origin_rrm: ViaUtils.Location.new_degrees(@default_latitude, @default_longitude),
      agl_m: nil,
      airspeed_mps: nil,
      rcin: @default_servo,
      servo_out: @default_servo,
      rc_passthrough: Keyword.get(config, :rc_passthrough, false),
      channel_names: Keyword.get(config, :channel_names, %{}),
      downward_range_max_m: Keyword.get(config, :downward_range_max_m, 0),
      downward_range_module: config[:downward_range_module],
      dt_accel_gyro_group: config[:dt_accel_gyro_group],
      gps_itow_position_velocity_group: config[:gps_itow_position_velocity_group],
      gps_itow_relheading_group: config[:gps_itow_relheading_group],
      airspeed_group: config[:airspeed_group],
      downward_range_distance_group: config[:downward_range_distance_group],
      publish_dt_accel_gyro_interval_ms: publish_dt_accel_gyro_interval_ms,
      publish_gps_position_velocity_interval_ms: publish_gps_position_velocity_interval_ms,
      publish_gps_relative_heading_interval_ms: publish_gps_relative_heading_interval_ms,
      publish_airspeed_interval_ms: publish_airspeed_interval_ms,
      publish_downward_range_distance_interval_ms: publish_downward_range_distance_interval_ms,
      exchange_data_loop_interval_ms: config[:sim_loop_interval_ms],
      exchange_data_watchdog: Watchdog.new(@clear_exchange_callback, 10000),
      exchange_data_timer: nil
    }

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

    ViaUtils.Process.start_loop(
      self(),
      state.publish_downward_range_distance_interval_ms,
      @down_range_loop
    )

    ViaUtils.Comms.join_group(__MODULE__, Groups.set_realflight_ip_address())
    ViaUtils.Comms.join_group(__MODULE__, Groups.get_realflight_ip_address())
    ViaUtils.Comms.join_group(__MODULE__, Groups.host_ip_address())
    ViaUtils.Comms.join_group(__MODULE__, Groups.virtual_uart_actuator_output())

    check_and_set_rf_ip_address(realflight_ip_address)
    {:ok, state}
  end

  @impl GenServer
  def terminate(reason, state) do
    Logger.error("#{__MODULE__} terminated for #{inspect(reason)}")
    state
  end

  @impl GenServer
  def handle_cast({Groups.host_ip_address(), host_ip_address}, state) do
    state =
      if !is_nil(host_ip_address) and !is_nil(state.realflight_ip_address) do
        Logger.debug("RFI Host IP Updated")
        Logger.debug("rfi ip: #{state.realflight_ip_address}")
        initialize_exchange_data(state)
      else
        state
      end

    {:noreply, %{state | host_ip_address: host_ip_address}}
  end

  @impl GenServer
  def handle_cast({Groups.set_realflight_ip_address(), realflight_ip_address}, state) do
    Logger.debug("RFI received RF IP: #{realflight_ip_address}")

    ViaUtils.File.write_file(@ip_filename, "/data/", realflight_ip_address)

    ViaUtils.Comms.cast_local_msg_to_group(
      __MODULE__,
      {Groups.realflight_ip_address(), realflight_ip_address},
      self()
    )

    url = realflight_ip_address <> ":#{@url_port}"
    state = %{state | url: url, realflight_ip_address: realflight_ip_address}

    state =
      if !is_nil(state.host_ip_address) do
        Logger.debug("RFI Host IP Updated")
        Logger.debug("rfi ip: #{state.realflight_ip_address}")
        Logger.debug("host ip: #{state.host_ip_address}")
        Process.sleep(1000)
        initialize_exchange_data(state)
      else
        state
      end

    {:noreply, state}
  end

  @impl GenServer
  def handle_cast({Groups.get_realflight_ip_address(), from}, state) do
    Logger.debug("RF rx get_rf_ip: #{state.realflight_ip_address}")

    GenServer.cast(from, {Groups.realflight_ip_address(), state.realflight_ip_address})

    {:noreply, state}
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

  @spec check_for_new_messages_and_process(list(), map()) :: map()
  def check_for_new_messages_and_process(data, state) do
    %{
      ubx: ubx,
      servo_out: servo_out_prev
    } = state

    {ubx, payload} = UbxInterpreter.check_for_new_message(ubx, data)

    if Enum.empty?(payload) do
      state
    else
      [aileron_prev, elevator_prev, throttle_prev, rudder_prev, _, flaps_prev, _, _, _, _, _, _] =
        servo_out_prev

      %{msg_class: msg_class, msg_id: msg_id} = ubx
      Logger.debug("msg class/id: #{msg_class}/#{msg_id}")

      state =
        case msg_class do
          ClassDefs.vehicle_cmds() ->
            case msg_id do
              BodyrateActuatorOutput.id() ->
                cmds =
                  UbxInterpreter.deconstruct_message_to_map(
                    BodyrateActuatorOutput.bytes(),
                    BodyrateActuatorOutput.multipliers(),
                    BodyrateActuatorOutput.keys(),
                    payload
                  )
                  |> Enum.reduce(%{}, fn {ch_name, value}, acc ->
                    one_sided_value = ViaUtils.Math.get_one_sided_from_two_sided(value)
                    Map.put(acc, ch_name, one_sided_value)
                  end)

                %{
                  Act.aileron() => aileron_scaled,
                  Act.elevator() => elevator_scaled,
                  Act.throttle() => throttle_scaled,
                  Act.rudder() => rudder_scaled
                } = cmds

                servo_out = [
                  aileron_scaled,
                  1-elevator_scaled,
                  throttle_scaled,
                  rudder_scaled,
                  0,
                  flaps_prev,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
                ]

                Logger.debug("servo out: #{ViaUtils.Format.eftb_list(servo_out, 3)}")
                %{state | servo_out: servo_out}

              ActuatorCmdDirect.id() ->
                cmds =
                  ActuatorCmdDirect.Utils.get_actuator_output(payload, state.channel_names)
                  |> Enum.reduce(%{}, fn {ch_name, value}, acc ->
                    one_sided_value = ViaUtils.Math.get_one_sided_from_two_sided(value)
                    Map.put(acc, ch_name, one_sided_value)
                  end)

                # Logger.debug("direct act: #{inspect(cmds)}")

                aileron_scaled = Map.get(cmds, Act.aileron(), aileron_prev)
                elevator_scaled = Map.get(cmds, Act.elevator(), elevator_prev)
                rudder_scaled = Map.get(cmds, Act.rudder(), rudder_prev)
                throttle_scaled = Map.get(cmds, Act.throttle(), throttle_prev)
                flaps_scaled = Map.get(cmds, Act.flaps(), flaps_prev)

                servo_out = [
                  aileron_scaled,
                  elevator_scaled,
                  throttle_scaled,
                  rudder_scaled,
                  0,
                  flaps_scaled,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
                ]

                Logger.debug("servo out: #{ViaUtils.Format.eftb_list(servo_out, 3)}")
                %{state | servo_out: servo_out}

              _other ->
                Logger.warn("Bad message id: #{msg_id}")
                state
            end

          _other ->
            Logger.warn("Bad message class: #{msg_class}")
            state
        end

      check_for_new_messages_and_process([], %{state | ubx: UbxInterpreter.clear(ubx)})
    end
  end

  @impl GenServer
  def handle_info({:circuits_uart, _port, data}, state) do
    Logger.debug("rfi rx: #{data}")
    state = check_for_new_messages_and_process(:binary.bin_to_list(data), state)
    Logger.debug("state servo out: #{ViaUtils.Format.eftb_list(state.servo_out, 3)}")
    {:noreply, state}
  end

  def handle_info(@start_exchange_data_loop, state) do
    exchange_data_timer =
      restart_exchange_data_timer(
        state.exchange_data_timer,
        state.exchange_data_loop_interval_ms
      )

    {:noreply, %{state | exchange_data_timer: exchange_data_timer}}
  end

  def handle_info(@exchange_data_loop, state) do
    state = exchange_data(state, state.servo_out)

    rcin =
      Enum.map(state.rcin, fn x ->
        ViaUtils.Math.get_two_sided_from_one_sided(x)
      end)

    ViaUtils.Comms.cast_global_msg_to_group(
      __MODULE__,
      {Groups.command_channels(), rcin},
      self()
    )

    {:noreply, state}
  end

  @impl GenServer
  def handle_info({@dt_accel_gyro_loop, dt_s}, state) do
    %{bodyaccel_mpss: bodyaccel_mpss, bodyrate_rps: bodyrate_rps, dt_accel_gyro_group: group} =
      state

    unless Enum.empty?(bodyaccel_mpss) or Enum.empty?(bodyrate_rps) do
      # Logger.debug("br: #{ViaUtils.Format.eftb_map_deg(bodyrate_rps, 1)}")

      ViaSimulation.Comms.publish_dt_accel_gyro(
        __MODULE__,
        dt_s,
        bodyaccel_mpss,
        bodyrate_rps,
        group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@gps_pos_vel_loop, state) do
    %{
      position_rrm: position_rrm,
      velocity_mps: velocity_mps,
      gps_itow_position_velocity_group: group
    } = state

    unless Enum.empty?(position_rrm) or Enum.empty?(velocity_mps) do
      ViaSimulation.Comms.publish_gps_itow_position_velocity(
        __MODULE__,
        position_rrm,
        velocity_mps,
        group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@gps_relhdg_loop, state) do
    %{attitude_rad: attitude_rad, gps_itow_relheading_group: group} = state

    unless Enum.empty?(attitude_rad) do
      %{SVN.yaw_rad() => yaw_rad} = attitude_rad

      ViaSimulation.Comms.publish_gps_relheading(
        __MODULE__,
        yaw_rad,
        group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@airspeed_loop, state) do
    %{airspeed_mps: airspeed_mps, airspeed_group: group} = state

    unless is_nil(airspeed_mps) do
      ViaSimulation.Comms.publish_airspeed(__MODULE__, airspeed_mps, group)
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@down_range_loop, state) do
    %{
      attitude_rad: attitude_rad,
      agl_m: agl_m,
      downward_range_distance_group: group,
      downward_range_max_m: downward_range_max_m,
      downward_range_module: downward_range_module
    } = state

    unless Enum.empty?(attitude_rad) or is_nil(agl_m) do
      range_m = ViaUtils.Motion.agl_to_range_measurement(attitude_rad, agl_m)

      if range_m < downward_range_max_m do
        ViaSimulation.Comms.publish_downward_range_distance(
          __MODULE__,
          range_m,
          downward_range_module,
          group
        )
      end
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@clear_exchange_callback, state) do
    Logger.warn("#{inspect(__MODULE__)} clear is_exchange_current}")
    url = state.url
    restore_controller(url)
    inject_controller_interface(url)

    {:noreply, %{state | exchange_data_watchdog: Watchdog.reset(state.exchange_data_watchdog)}}
  end

  def initialize_exchange_data(state) do
    Logger.debug("EFI init ex data")

    reset_realflight_interface()

    ViaUtils.Comms.cast_local_msg_to_group(
      __MODULE__,
      {Groups.realflight_ip_address(), state.realflight_ip_address},
      self()
    )

    :erlang.send_after(1000, self(), @start_exchange_data_loop)
    state
  end

  def reset_realflight_interface() do
    Logger.debug("reset realflight interface")
    restore_controller()
    inject_controller_interface()
  end

  def restart_exchange_data_timer(exchange_data_timer, interval_ms) do
    Logger.info("RFI start loops: #{interval_ms}")

    if is_nil(exchange_data_timer) do
      ViaUtils.Process.start_loop(
        self(),
        interval_ms,
        @exchange_data_loop
      )
    else
      exchange_data_timer
    end
  end

  def check_and_set_rf_ip_address(fallback_ip) do
    Logger.debug("RFI checking for rf ip")

    realflight_ip_binary =
      ViaUtils.File.read_file_target(
        @ip_filename,
        ViaUtils.File.default_mount_path(),
        ViaUtils.File.target?()
      )

    Logger.debug("RFI ip = #{realflight_ip_binary}")

    realflight_ip =
      cond do
        !is_nil(realflight_ip_binary) -> realflight_ip_binary |> String.trim_trailing("\n")
        !is_nil(fallback_ip) -> fallback_ip
        true -> raise "No valid Realflight IP address available"
      end

    Logger.debug("RFI Realflight IP found: #{inspect(realflight_ip)}")
    GenServer.cast(__MODULE__, {Groups.set_realflight_ip_address(), realflight_ip})
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
    Logger.debug("restore controller")
    post_poison(url, body, 1000)
  end

  @spec inject_controller_interface(binary()) :: binary()
  def inject_controller_interface(url) do
    body = "<?xml version='1.0' encoding='UTF-8'?>
    <soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
    <soap:Body>
    <InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
    </soap:Body>
    </soap:Envelope>"
    Logger.debug("inject controller interface")
    post_poison(url, body, 1000)
  end

  @spec exchange_data(map(), list()) :: map()
  def exchange_data(state, servo_output) do
    %{
      url: url,
      position_origin_rrm: position_origin,
      servo_out: servo_out_prev,
      rc_passthrough: rc_passthrough,
      exchange_data_watchdog: exchange_data_watchdog
    } = state

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
    response = post_poison(url, body)

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
      position = Utils.extract_position(aircraft_state, position_origin)
      velocity = Utils.extract_velocity(aircraft_state)
      attitude = Utils.extract_attitude(aircraft_state)
      bodyaccel = Utils.extract_bodyaccel(aircraft_state)
      bodyrate = Utils.extract_bodyrate(aircraft_state)
      agl = Utils.extract_agl(aircraft_state)
      airspeed = Utils.extract_airspeed(aircraft_state)
      rcin = Utils.extract_rcin(rcin_values)
      servo_out = if rc_passthrough, do: rcin, else: servo_out_prev

      # Logger.debug("rcin: #{inspect(rcin)}")
      # Logger.debug("position: #{ViaUtils.Location.to_string(position)}")
      # Logger.debug("velocity: #{ViaUtils.Format.eftb_map(velocity, 2)}")
      # Logger.debug("attitude: #{ViaUtils.Format.eftb_map_deg(attitude, 1)}")
      # Logger.debug("bodyaccel: #{ViaUtils.Format.eftb_map(bodyaccel, 3)}")
      # Logger.debug(ViaUtils.Format.eftb_map_deg(bodyrate, 2))
      # Logger.debug("agl: #{ViaUtils.Format.eftb(agl,2)}")
      # Logger.debug("airspeed: #{ViaUtils.Format.eftb(airspeed,2)}")

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
          servo_out: servo_out,
          exchange_data_watchdog: Watchdog.reset(exchange_data_watchdog)
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
end
