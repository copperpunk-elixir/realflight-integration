defmodule ViaUtils.Process do
  require Logger

  @moduledoc """
  Documentation for `UtilsProcess`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> UtilsProcess.hello()
      :world

  """
  def start_link_redundant(parent_module, module, config, name \\ nil) do
    name =
      case name do
        nil -> module
        atom -> atom
      end

    result =
      case parent_module do
        GenServer -> GenServer.start_link(module, config, name: name)
        Supervisor -> Supervisor.start_link(module, config, name: name)
        DynamicSupervisor -> DynamicSupervisor.start_link(module, config, name: name)
        Registry -> apply(Registry, :start_link, [config])
        Agent -> Agent.start_link(fn -> config end, name: name)
      end

    case result do
      {:ok, pid} ->
        # Logger.debug("#{module}: #{inspect(name)} successfully started")
        wait_for_genserver_start(pid)
        {:ok, pid}

      {:error, {:already_started, pid}} ->
        # Logger.debug("#{module}: #{inspect(name)} already started at #{inspect(pid)}. This is fine.")
        {:ok, pid}

      other ->
        raise "#{inspect(other)} result from start_link_redundant"
    end
  end

  def start_link_singular(parent_module, module, config, name \\ nil) do
    name =
      case name do
        nil -> module
        atom -> atom
      end

    result =
      case parent_module do
        GenServer -> GenServer.start_link(module, config, name: name)
        Supervisor -> Supervisor.start_link(module, config, name: name)
        DynamicSupervisor -> DynamicSupervisor.start_link(module, config, name: name)
        Registry -> apply(Registry, :start_link, [config])
        Agent -> Agent.start_link(fn -> config end, name: name)
      end

    case result do
      {:ok, pid} ->
        # Logger.debug("#{module}: #{inspect(name)} successfully started")
        wait_for_genserver_start(pid)
        {:ok, pid}

      {:error, {:already_started, pid}} ->
        raise "#{module}: #{inspect(name)} already started at #{inspect(pid)}. This is not okay."
        {:error, pid}
    end
  end

  def wait_for_genserver_start(process_name, current_time \\ 0, timeout \\ 60000) do
    # Logger.debug("Wait for GenServer process: #{inspect(process_name)}")
    if GenServer.whereis(process_name) == nil do
      if current_time < timeout do
        Process.sleep(100)
        wait_for_genserver_start(process_name, current_time + 10, timeout)
      else
        Logger.error("Wait for GenServer Start TIMEOUT. Waited #{timeout / 1000}s")
      end
    end
  end

  def start_loop(process_id, loop_interval_ms, loop_callback) do
    case :timer.send_interval(loop_interval_ms, process_id, loop_callback) do
      {:ok, timer} ->
        # Logger.debug("#{inspect(loop_callback)} timer started!")
        timer

      {_, reason} ->
        Logger.warn("Could not start #{inspect(loop_callback)} timer: #{inspect(reason)} ")
        nil
    end
  end

  def stop_loop(timer) do
    case :timer.cancel(timer) do
      {:ok, _} ->
        nil

      {_, reason} ->
        Logger.warn("Could not stop #{inspect(timer)} timer: #{inspect(reason)} ")
        timer
    end
  end

  def attach_callback(process_id, loop_interval_ms, timer_callback) do
    case :timer.send_after(loop_interval_ms, process_id, timer_callback) do
      {:ok, timer} ->
        # Logger.debug("#{inspect(timer_callback)} timer started!")
        timer

      {_, reason} ->
        Logger.warn("Could not start #{inspect(timer_callback)} timer: #{inspect(reason)} ")
        nil
    end
  end

  def detach_callback(timer) do
    case :timer.cancel(timer) do
      {:ok, _} ->
        nil

      {_, reason} ->
        Logger.warn("Could not stop #{inspect(timer)} timer: #{inspect(reason)} ")
        timer
    end
  end

  @spec reattach_timer(any(), integer(), any()) :: any()
  def reattach_timer(timer, interval_ms, callback) do
    detach_callback(timer)

    attach_callback(
      self(),
      interval_ms,
      callback
    )
  end
end
