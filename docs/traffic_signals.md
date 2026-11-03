# Traffic signals

An engine-neutral subsystem: fixed-time signal plans derived from the served junction furniture, a pure state function
of (plan, time), a position index and the stop-or-go rule. It exists so that every agent that must obey signals reads the
same lamps: the racing game's NPC traffic, the farm game's helper drivers, a future server that mirrors state to clients.
It has no Godot, racing_game or farming_sim dependency; it links only with the layer library
(`include/g2m/layer/signal_plan.h`, `src/layer/signal_plan.cpp`, data `data/signals/signal_timing.json`).

## What it adds, and what it does not

Added: a plan per signalised node (phases, cycle, offset), the lamp state at any time, the stop line and signal heads of each
controlled approach as world geometry, `SignalIndex::next_ahead` ("which stop line is this agent approaching, and what shows
on it"), `signal_advice` (stop or go) and `signal_stop_speed_cap`.

Not added: lamp meshes and lamp animation (a renderer stands a head on each `SignalHead` and colours it by the aspect),
pedestrian signals, protected left turns, detector-actuated control, green waves between neighbouring signals, signal
complexes (a junction whose arms carry their own crossing signals gets one plan per node, each with its own offset), and
the NPC/helper obedience itself: the consumers call the API below.

## Data flow

```
OSM node highway=traffic_signals (or a signalised crossing)
  -> roads.geom furniture (road_furniture.cpp: one StopLine record per lane group, one Signal record per head)
  -> build_signal_plans(tile)              no new roads.geom encoding, no release id change
  -> SignalIndex::add(plans)               one index per consumer, any number of tiles
  -> SignalIndex::next_ahead(...) / SignalPlan::state(...)
```

Nothing is baked: the plans are a pure function of `RoadGeomTile::furniture` and `data/signals/signal_timing.json`, so they
are rebuilt from the tiles a consumer already loads. A junction's furniture is emitted only by its owner tile and a
mid-block signal's by every tile that holds its entry, identical in each; `SignalIndex::add` skips a plan whose node is
already present.

## Plan model

- **Approach**: one controlled direction of one road: the union of its stop-line lane groups (`ax,ay`-`bx,by`, centre `x,y`),
  the unit travel direction, the lane count, and one or two heads (the second on the travel-left side, from the furniture
  rule). Identity `id()` = FNV-1a of (node, way, travel, station mm).
- **Phases**: approaches on one road axis (opposed or parallel, `|cross| <= 0.5`) share a phase; the phases are ordered by the
  axis angle. A signal whose approaches lie on one axis (a mid-block crossing signal) gets a virtual cross phase without
  approaches, so the road is stopped once per cycle.
- **Timing** (all data, `signal_timing.json`): phase = red-amber 1 s, green = `12 s + 6 s * lanes` of its widest approach
  clamped to 8..40 s, amber 3 s, clearance 2 s; the cycle is scaled down to 120 s if longer (never below 8 s of green).
  The cycle offset is the node id hash modulo the cycle, so neighbouring signals are not in step.
- **State**: `SignalPlan::state(phase, t_s)` returns the aspect (`Red`, `RedAmber`, `Green`, `Amber`) and the seconds until it
  changes. Integer millisecond arithmetic; `t_s` is any absolute clock. **All consumers must pass the same clock** (the world's
  environment time or the server tick time); then the lamps agree everywhere and nothing needs to be sent over the network.
  No two phases are ever green (or red-amber) together; test S2 scans a cycle.

## Using it (NPCs and farm helpers)

```cpp
SignalIndex signals;                                   // once
signals.add(*build_signal_plans(road_geom_tile));      // per loaded tile

// every decision tick of an agent at (x, y) heading (hx, hy), speed v:
if (auto hit = signals.next_ahead(x, y, hx, hy, /*horizon_m=*/80, world_time_s, rules)) {
    if (signal_advice(*hit, v, rules) == SignalAdvice::Stop)
        speed_cap = signal_stop_speed_cap(hit->distance_m, rules);   // approach the stop line smoothly
}
```

`SignalDriverRules` carries the comfortable (3 m/s^2) and maximum (6 m/s^2) deceleration, the reaction time (0.5 s), the lateral
tolerance and the heading tolerance; a heavy vehicle (a tractor with a loaded trailer) passes its own values (a smaller
comfortable deceleration makes it stop earlier on amber). The rule: green goes; amber stops if the agent can still stop at
the line with the comfortable deceleration after its reaction time, otherwise it goes (the dilemma zone); red and red-amber
stop; a line already passed goes on.

`next_ahead` identifies the approach geometrically (the agent lies behind a stop line, within its ends plus the tolerance,
heading along its travel direction), so an agent needs no road graph. An agent that follows a route may instead look the
plan up by node (`plan_of_node`) and the approach by way and direction.

## Decisions

**D1. Plans are derived from furniture records at load time, not stored in roads.geom.** Alternatives: (a) a new roads.geom
section with the plans (explicit, but a format change and a release id change for every tile, and the plan depends only on
records already served); (b) derive per consumer as racing_game does today (a two-phase heuristic from heading differences
inside the game; the farm game would copy it). Chosen: derive in the library from the served records, so there is one
implementation, no re-bake, and the result is a pure function of served data. Cost: each consumer rebuilds the plans of the
tiles it loads (microseconds per junction).

**D2. State is a pure function of (plan, time), not a simulated state machine.** Alternatives: tick each controller (needs
per-world state, snapshot and restore, hash folding, network sync); detector-actuated control (realistic, not deterministic
without the full traffic state). Chosen: fixed-time plans evaluated from the clock: deterministic, restartable, tile-invariant,
no server traffic. Cost: no adaptive behaviour.

**D3. Phases by road axis, not by turn movements.** A two-phase plan (more phases for odd junctions) is what a junction
without protected turns does in Germany. Left turners yield to oncoming traffic as in normal driving. Alternatives: signal
group tables per junction from OSM `turn:lanes` and relations (unavailable for most junctions in OSM) or from imagery.

**D4. Timing is data.** Durations, the lane-dependent green, the cycle cap and the driver's deceleration values are in
`data/signals/signal_timing.json` with sources ([D] RiLSA 2010 for amber and red-amber, the rest [assumed]), strict loader and
range checks, like the other rule files. The file is not part of any layer config hash.

**D5. One module for NPCs and helpers.** The farm helpers and the NPCs differ only in the `SignalDriverRules` they pass and
in the route they follow; the module knows neither.
