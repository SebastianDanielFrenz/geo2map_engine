# terrain.carved — layer `g2m.terrain.carved` v1 (body encoding 1)

The terrain height pyramid with the served road geometry carved in (street fidelity P2 slice 9, design
[../street_fidelity/p2_junction_design.md](../street_fidelity/p2_junction_design.md) sections 5.3 and 6.1-6.4). It is a
separate layer: `g2m.terrain.height` stays the bare DGM1 pyramid and the default; a consumer opts in by requesting this
layer. Code: `include/g2m/layer/terrain_carve.h`, `src/layer/terrain_carve.cpp`.

## Bytes

No new codec. A tile is a `g2m.tile/1` container ([tile_container.md](tile_container.md)) whose body is the height tile of
body encoding 1 ([src_elev.md](src_elev.md)), exactly as `g2m.terrain.height`: 256 × 256 `int32` samples in 1/256 m,
cell-registered, row 0 = south, NoData = `INT32_MIN` (`encode_height_tile` / `decode_height_tile`, `height_tile.h`).

| Field | Value |
|---|---|
| Layer id | `g2m.terrain.carved`, version 1, kind Baked |
| Levels | 0 to 6; level L covers 256 · 2^L m with 2^L m cells |
| Encoding | 1 (SrcElev), encoding version 1 |
| Header datum | level 0: the datum of the `g2m.elev.base` inputs (DHHN2016 or EGM2008; mixed datums, or a `roads.geom` tile in another datum, are rejected); levels 1-6: DHHN2016, as `g2m.terrain.height` |
| Header height offset | 0 (input offsets are applied once, before carving) |
| Config hash | SHA-256 of `g2m.terrain.carved/1 smooth+carve L0 roads.geom enc=7 radius=192 carve=<sha256 of data/carve/carve_default.json>; L1-L6 mean-of-4 half-even` (`terrain_carved_config_text()`) |

## Inputs

- **Level 0**, tile Q: the 9 `g2m.elev.base` level-0 tiles Q + (dx, dy), dx, dy ∈ {−1, 0, 1} (absent = NoData; an absent
  centre fails with NotFound), and the `g2m.roads.geom` level-2 tiles intersecting Q's square grown by 192 m on every side,
  half-open (`carve_road_geom_keys`: one tile typically, two along a level-2 edge, four at a corner; absent = no roads).
  The list is fixed by the key alone; `derive` rejects any other list.
- **Levels 1-6**: the 4 children of the same layer one level down (a missing child counts as all NoData).

## Values

Level 0, per cell with centre (E + 0.5, N + 0.5) in zone metres:

1. **T** = the DEM smoothed by the ported game kernel: per axis Σ k·h / Σ k over the taps that are not NoData with
   k = [138, 621, 1024, 621, 138], rounded half-even on int64, then `h + round_half_even(35 · (s − h) / 100)`. A NoData cell
   stays NoData.
2. **Cores.** Inside the outer ring of a carved junction (not declined, not `OnStructure`): the junction surface
   (`junction_height_m`); junction cores beat approach cores; among several, the lowest z. Else, on a ground approach between
   its trims and within its outer edges (`carve_footprint`): `road_surface_height_m`; the lowest z.
3. **Blends.** Else the blend of largest weight w = 1 − S(d/b), ties to the lowest z, value z_b + (T − z_b) · S(d/b), with
   S the smooth step, d the distance to an approach segment's band rectangle or to a junction ring edge (design D40), z_b the
   core height at the nearest boundary point and b = clamp(max(b_class, 3 · |T_edge − z_edge|), 3 m, 15 m).
4. Else **T**.
5. **Deck clamp.** Within a bridge's paved band + 1 m: z = min(z, deck_top − 0.55 · S(e / 4 m)), e the station distance from
   the nearer bridge end. It only lowers.
6. `round_half_even(z · 256)`.

Bridges and tunnels are never carved. Parameters (kernel, strength, blend widths per class, deck clearance) come from
`data/carve/carve_default.json`; its SHA-256 is part of the config hash. Every value is a function of the world position,
so neighbouring tiles agree at their shared edge without sharing samples.

Levels 1-6: `downsample_2x2` of the 4 children: mean of 4, rounded half-even; NoData if any source sample is NoData.

## Determinism

Integer smoothing and quantisation are exact; the surfaces and blends are double arithmetic in a fixed order with + − × ÷
and `det::sqrt` only (no libm, no FMA), and no unordered iteration decides a value. The result is the same for every worker
count and on Windows and Linux (golden `testdata/golden/g3d_golden.txt`).
