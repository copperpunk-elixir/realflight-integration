defmodule ViaUtils.Watchdog do
  defstruct [:timer, :duration_ms, :callback]

  @spec new(any()) :: struct()
  def new(callback) do
    %ViaUtils.Watchdog{callback: callback}
  end

  @spec new(any(), integer()) :: struct()
  def new(callback, duration_ms) do
    %ViaUtils.Watchdog{callback: callback, duration_ms: round(duration_ms)}
  end

  @spec reset(struct()) :: struct()
  def reset(watchdog) do
    if is_nil(watchdog.duration_ms), do: raise "Watchdog #{inspect(watchdog.callback)} duration_ms has not been set."
    timer = ViaUtils.Process.reattach_timer(watchdog.timer, watchdog.duration_ms, watchdog.callback)
    %{watchdog | timer: timer}
  end

  @spec reset(struct(), integer()) :: struct()
  def reset(watchdog, duration_ms) do
    timer = ViaUtils.Process.reattach_timer(watchdog.timer, round(duration_ms), watchdog.callback)
    %{watchdog | timer: timer}
  end
end
