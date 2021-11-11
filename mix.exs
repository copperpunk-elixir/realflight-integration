defmodule RealflightIntegration.MixProject do
  use Mix.Project

  @version "0.1.2"
  @source_url "https://github.com/copperpunk-elixir/realflight-integration"

  def project do
    [
      app: :realflight_integration,
      version: @version,
      elixir: "~> 1.12",
      description: description(),
      package: package(),
      source_url: @source_url,
      docs: docs(),
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger]
    ]
  end

  defp description do
    "GenServer to integrate with RealFlight simulator. Build to assist the Via autopilot."
  end

  defp package do
    %{
      licenses: ["GPL-3.0"],
      links: %{"Github" => @source_url}
    }
  end

  defp docs do
    [
      extras: ["README.md"],
      main: "readme",
      source_ref: "v#{@version}",
      source_url: @source_url
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:ex_doc, "~> 0.24", only: :dev, runtime: false},
      # "~> 0.2.0"},
      {:via_utils,
       path: "/home/ubuntu/Documents/Github/cp-elixir/libraries/via-utils", override: true},
      {:via_simulation, path: "/home/ubuntu/Documents/Github/cp-elixir/libraries/via-simulation"},
      {:via_telemetry, path: "/home/ubuntu/Documents/Github/cp-elixir/libraries/via-telemetry"},
      #  git: "https://github.com/copperpunk-elixir/via-utils.git", tag: "v0.1.4-alpha"},
      {:soap, "~> 1.0.1"},
      {:httpoison, "~> 1.8.0"},
      {:sax_map, "~> 1.0"}
    ]
  end
end
