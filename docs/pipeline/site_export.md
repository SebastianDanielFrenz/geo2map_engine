# Site export: candidates for buyers and supply stores

Detail page of the [pipeline overview](README.md), section 2.9. Not a layer and not baked: a dev/bake-time tool over an existing source store.

```
g2m_site_export --store DIR --scope SCOPE --rules data/site_export/de.json --out sites.json [--region-min E,N --region-max E,N]
```

## What it writes

`farm.site_input/1` (the farming game's `docs/farm/economy.md`, "Buyer sites (X8)" and "Supply sites (X9)"): 

- `features`: every OSM **node or way** that carries a tag listed in the rules file or has a listed word in its `name` (case-insensitive). Each record has `id`, **all** tags, `name`, and the position `e_mm`/`n_mm` in integer UTM millimetres of the bake's zone (a way takes the mean of its nodes). The consumer classifies; the tool only filters. Ids: a node keeps its OSM id, a way's is its id with bit 60 set; the top bit is left to the consumer's procedural sites. Relations are not exported.
- `industrial`: `landuse=industrial` ways (mean position) with `road_mm`, the distance to the nearest `highway=*` way vertex of the same tile (1e9 mm when the tile has none). The consumer puts procedural sites on the best candidates.
- `region`: the bounding box of the scope's tiles unless `--region-min/--region-max` (metres) are given.

Tile halos repeat features; each id is kept once. Output is sorted by id and the same bytes on every run. The rules (`data/site_export/de.json`, `g2m.site_export_rules/1`) list tags and name words with their sources; they mirror the farm's `buyers.json`/`supply.json` and need a change only when the farm starts to classify on a new tag.

## Home region numbers (`home-2026-09r1`)

1085 tiles, 180 features, 254 industrial candidates, 52 KB. A farm classification of the result is in the farm's economy doc.
