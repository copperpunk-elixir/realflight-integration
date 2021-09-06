defmodule ViaUtils.Helper.Test do
  def prepare_test() do
    RingLogger.attach()
    ViaUtils.Registry.start_link()
  end

end
