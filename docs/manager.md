# geo2map_engine work tracker

This file goes with the manager PR, a standing draft PR that coordinates open issues and feature requests for this repository. The owner asked for it on racing_game PR #5 (2026-10-15).

How it works:
- **Requests and questions** go into a comment on the manager PR.
- **Real work** happens in its own PR. Each item below links to that PR once it exists. The manager PR is never the place where code lands.
- **This file** is updated whenever an item changes state, so the PR diff always shows the current list.
- **Owner decisions** are recorded here with the date, and also in the design doc the decision belongs to. Decisions keep their D-number (for example D52).

Last updated: 2026-10-16.

## In progress

| Item | Where | State |
|---|---|---|
| G4 routing slice 3: turn restrictions in `roads.graph` (encoding 6), via-way rules, one release-id bump | branch claude/route-slice3 | being built |

## Waiting on the owner

| Item | What is needed |
|---|---|
| Underpass census on the home store (G-6) | One run of `g2m_road_probe <store> <scope> <level-2 tile> out.csv --underpasses` on the warmed home store. It tells us how many real crossings the underpass mask touches. |
| Phase 7 carve switch ([terrain_migration_plan.md](terrain_migration_plan.md#phase-7-carve-switch-id-bump-3-l)) | The real-store roadcmp probe after warming the stores. The game deletes its own profile completion only once the probe shows zero game-only entries (standing rule). |
| racing_game over HTTP? | PR #8 (G6) left this open: should the game get an opt-in `g2m_server` sidecar over HTTP instead of the in-process server? |

## Next up

| Item | Notes |
|---|---|
| G4 routing follow-ups | Lane hints in guidance and timing on the real store ([routing_design.md](routing_design.md)). |
| Long tunnels split into several OSM ways | D52 judges each stretch by its own length. A long tunnel made of several short ways keeps DEM heights at its inner joins. The owner kept the per-stretch default (#11); revisit once above-ground roads look right. |
| Koenigstein `roads.geom` tile rejected by the deriver | Tile `utm32n/2/452/5428`. racing_game falls back to smoothed terrain there (`RG_ROAD_SURFACE unavailable reason=deriver_rejected`). Find out why the deriver rejects it and fix it. |
| Consumer switches in racing_game | Render class window from `g2m.terrain.class`; served carved heights (phase 7). |
| G3-E / G3-F | Ribbons, markings and bridge bodies; route acceptance on a 20-junction route. |
| P2 slices 10-12, P3 markings | [street_fidelity/p2_junction_design.md](street_fidelity/p2_junction_design.md), [street_fidelity/plan.md](street_fidelity/plan.md). |
| G5 land cover, buildings, water, vegetation | racing_game brief lane E2. |
| G7 hosted server | Built on the accepted G6 binding: TLS through a reverse proxy, auth and rate limits, HTTP upstream chaining, "503 while queued" for long cold derivations, and a CDN / static-pack upload path. |

## Documentation debt

These items are open in [pipeline/README.md](pipeline/README.md) section 9:
- 5: stale parts of CLAUDE.md;
- 6: PLAN.md vs code for terrain;
- 7: PLAN.md vs code for the CLI;
- 9: unifying the height quantiser;
- 11: physics_sim pin values;
- 12: whether roads are baked for the home store;
- 14: whether curvature exceptions are decided.

## Done recently

| Date | Item | PR |
|---|---|---|
| 2026-10-15 | Underpass decks over lower roads in a cutting, steep walls, tag rules for parking structures and covered tunnels (D53-D61); racing_game #16 pinned to main | #13 |
| 2026-10-15 | Long tunnels follow one straight grade between their portals (D52, threshold 200 m) | #11 |
| 2026-10-15 | Junction patches split between sett and paving-stone arms; sett ribbon flag | #9, #10 |
| 2026-10-15 | G6: protocol v1 over HTTP and the `g2m_server` sidecar | #8 |
| 2026-10-14/15 | Terrain migration phases 3a-6 | #3-#7 |
