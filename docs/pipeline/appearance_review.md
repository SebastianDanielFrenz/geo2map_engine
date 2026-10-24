# Appearance comparison fixture

`g2m_appearance_preview OUTPUT.json` (built with the tiler applications) exports
native building meshes and bridge safety barriers as `g2m.appearance_preview/1`.
It covers residential, commercial and industrial facade archetypes, explicit
material overrides, and motorway, residential-road and pedestrian parapets.
The fixtures are inferred appearance examples, not surveyed buildings.

In racing_game, set `RG_APPEARANCE_FIXTURE` to the absolute JSON path and
`RG_APPEARANCE_IMAGE` to the destination PNG, then run Godot with
`--script res://scripts/appearance_review.gd --resolution 1280x950`.
Use a rendering-capable instance, not the headless dummy renderer. The gallery
uses the shipping building material loader and native mesh coordinates. It
renders eight buildings and three barrier styles before saving and exiting.

Geometry regressions in `[building_mesh]` and `[bridge_visual]` verify bounds,
material precedence, floor placement and physical-deck separation. The gallery
is an additional visual review; passing those tests alone does not establish
that the graphical target has been met.
