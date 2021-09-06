defmodule ViaUtils.Helper.Process.GenServer do
  use GenServer
  require Logger

  def start_link(_config) do
    ViaUtils.Process.start_link_singular(GenServer, __MODULE__, nil)
  end

  @impl GenServer
  def init(_) do
    {:ok, %{}}
  end

  @impl GenServer
  def handle_info(:timer_callback, state) do
    Logger.debug("timer callback at #{:erlang.monotonic_time(:microsecond)}")
    {:noreply, state}
  end
end
