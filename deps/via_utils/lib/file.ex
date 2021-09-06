defmodule ViaUtils.File do
  @moduledoc """
  Documentation for `ViaUtils.File`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> UtilsFile.hello()
      :world

  """
  require Logger

  defmacro default_mount_path, do: "/mnt"

  @spec mount_usb_drive(binary(), binary()) :: atom()
  def mount_usb_drive(drive_location, mount_path \\ default_mount_path()) do
    drive_location = "/dev/" <> drive_location

    {_resp, error_code} = System.cmd("mount", [drive_location, mount_path])

    if error_code == 0 do
      :ok
    else
      raise "Device at #{drive_location} could not be mounted to #{mount_path}"
    end
  end

  @spec unmount_usb_drive(binary()) :: tuple()
  def unmount_usb_drive(mount_path \\ default_mount_path()) do
    Logger.debug("Unmount USB drive from #{mount_path}")
    System.cmd("umount", [mount_path])
  end

  @spec cycle_mount(binary(), binary()) :: atom()
  def cycle_mount(drive_location, mount_path \\ default_mount_path()) do
    Logger.debug("Cycling usb drive mount")
    unmount_usb_drive(mount_path)
    Process.sleep(500)
    mount_usb_drive(drive_location, mount_path)
    :ok
  end

  @spec get_filenames_with_extension(binary(), binary(), binary()) :: list()
  def get_filenames_with_extension(
        extension,
        subdirectory \\ "",
        mount_path \\ default_mount_path()
      ) do
    path = mount_path <> "/" <> subdirectory
    {:ok, files} = :file.list_dir(path)

    filenames =
      Enum.reduce(files, [], fn file, acc ->
        file = to_string(file)

        if String.contains?(file, extension) do
          [filename] = String.split(file, extension, trim: true)
          acc ++ [filename]
        else
          acc
        end
      end)

    filenames
  end
end
