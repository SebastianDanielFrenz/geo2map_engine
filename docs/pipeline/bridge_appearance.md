# Bridge appearance

Bridge appearance is a render-only mesh built from an authoritative road deck. It reads the deck edge rows, leaves the deck mesh and colliders unchanged, and adds generic parapets when the deck is not inferred or a tunnel floor.

Parapet rails follow the sampled deck edge. Chord intervals are subdivided until the measured three-dimensional deviation of every intervening deck row is at most 0.05 m. This leaves clearance for the 0.22 m outward offset and rail thickness. The original maximum 80 base panels preserve straight-bridge post spacing; curvature can refine them up to a hard limit of 512 panels. Invalid input or an exceeded panel or mesh budget reports the visual as unsupported and returns no partial mesh.

Explicit beam/slab, truss, suspension, and cable-stayed forms remain procedural visual approximations. When full-span sampled deck deviation exceeds 0.05 m, straight truss and cable layouts are suppressed and the visual reports unsupported while retaining only the safe parapets. The generic `bridge:structure=arch` tag does not say whether the arch is above, through, or below the deck. The tag remains in provenance; until a source identifies topology, the renderer emits generic parapets and reports the structure-specific body as unsupported.

No piers are inferred in lower roads or water, and visual members do not become physical deck or collider geometry.
