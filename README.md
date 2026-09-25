# geo2map_engine

An engine-neutral C++20 library and map server that turn real-world geodata
into streamable game-map layers: terrain, roads with a routable graph, land
cover and buildings. It is also the map server itself: in-process, as a local
sidecar, or hosted, all behind one protocol.

- Consumers: racing_game (Godot); later a farming sim and UE5 via the flat C API.
- The design and phase plan is in `PLAN.md`.
- Licence: MIT (see `LICENSE`). Data derived from OpenStreetMap is ODbL 1.0,
  "(c) OpenStreetMap contributors". Other data sources carry their own
  licences (PLAN.md section 5).

## Building

Toolchain: `tools\setup_dev_env.ps1 -CheckOnly`. Build, test and CI commands
are in `CLAUDE.md` ("Building and testing", "CI"); the full CI is
`tools\ci.ps1`. Third-party code and licences: `THIRD_PARTY_NOTICES.md`.
