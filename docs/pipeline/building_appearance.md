# Procedural building appearance

`build_building_mesh` turns the footprint and appearance tags in a `Building`
into engine-neutral render triangles. It is a visual consumer helper: it does
not change the extracted footprint, reported height, ground sampling, collision
geometry, or any served tile.

## Facade archetypes

The `building=*` type selects a restrained procedural style. The style changes
window proportions and spacing, frame and glass colours, and where supported
adds a base plinth, floor bands, mullions, and a single entrance. Detached and
generic low-rise residential buildings use a plaster-like palette and a regular
window rhythm. Townhouses use narrower openings and closer bays. Apartments use
denser windows, floor bands, mullions, and an entrance. Commercial buildings
use wide ground-floor glazing with a separate upper-floor rhythm. Industrial
and warehouse buildings use fewer, higher windows, stronger bands, and an
entrance. Other types use the generic low-rise style. These are visual
inferences from broad OSM categories, not claims about measured facades.

Each building ID and mesh seed select one of three coherent facade variants.
The variant modestly adjusts window width, bay spacing, and frame finish across
the whole building, so neighboring houses avoid a repeated mould without
random differences from window to window. Repeating a request with the same
input gives the same geometry and colours.
`facade_detail=false` skips windows, doors, plinths, floor bands, and roof
ornaments while retaining the building walls and roof silhouette.

## Explicit tags and material IDs

`building:colour` sets the wall tint when it parses as a supported colour and
takes precedence over the archetype palette. Recognized `building:material`
values also take precedence over the archetype wall material: brick/masonry
maps to `Brick`, stone variants to `Stone`, wood/timber to `Timber`, concrete to
`WallSmooth`, metal/steel/aluminium to `Cladding`, and plaster/render to `Wall`.
Textured facade IDs use a light neutral vertex tint so their material texture
keeps its intended brightness. Plaster `Wall` facades use the archetype palette
with a small whole-building variation. An explicit wall colour overrides either
fallback. Unrecognized material values leave the archetype choice in effect.
Existing roof shape, colour, material, orientation, and direction handling is
unchanged.

`BuildingMaterial` IDs 0 through 7 remain `Wall`, `Roof`, `Glass`, `Frame`,
`Brick`, `Slate`, `FlatRoof`, and `Metal`. Facade IDs are appended: `Stone` 8,
`Timber` 9, `WallSmooth` 10, `Cladding` 11, `Door` 12, `OfficeWindow` 13,
and `IndustrialWindow` 14; `Count` is 15. Residential and generic windows use
`Glass`; commercial, retail, and office windows share `OfficeWindow`; industrial,
warehouse, and factory windows share `IndustrialWindow`. A near tile can add at
most two window material batches when these archetypes are present, regardless
of how many windows it contains. Render bindings can size batches from `Count`.

Window surfaces (`Glass`, `OfficeWindow`, `IndustrialWindow`) and opaque door
panels (`Door`) use one upright unit quad each, with image-origin UVs:
bottom-left `(0,1)`, bottom-right `(1,1)`, top-right `(1,0)`, and top-left
`(0,0)`. This keeps decoded texture images upright when UV `(0,0)` addresses
the image's top-left. They use a white vertex tint for atlas colour. The door
retains its frame and handle but uses a single front panel instead of a solid
box insert. These atlas surfaces keep six vertices per quad;
`facade_detail=false` emits no window, door, or facade frame detail.

## Geometry and bounds

Window count remains limited by `BuildingMeshOptions::max_windows`, capped at
4096 per building. Floors and bays are capped at 64 for each ring edge. The
existing total-height bound, courtyard openings, concave roof clipping, mapped
roof directions, and normal generation continue to use the input footprint
and roof metadata. Facade details are render-only and do not add colliders.

The appearance checks live in `tests/world/test_building_mesh.cpp` and cover
archetype rhythm, distinct window heights across floors, deterministic house
variants, material IDs, explicit material/colour precedence, the detail-free
path, and deterministic mesh output.
