defmodule RealflightIntegration.MixProject do
  use Mix.Project

  def project do
    [
      app: :realflight_integration,
      version: "0.1.0",
      elixir: "~> 1.12",
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

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:via_utils, "~> 0.1.4"},
      {:soap, "~> 1.0.1"},
      {:httpoison, "~> 1.8.0"},
      {:sax_map, "~> 1.0"}
    ]
  end
end
