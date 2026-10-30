# Terrarium elevation sampling

`g2m::decode::TerrariumDem` samples a bounded, in-memory mosaic of decoded
Terrarium XYZ tiles. It accepts tightly packed row-major RGBA8 pixel buffers;
image decoding, WebP support, file access, and network fetching remain with the
caller. The sampler is engine-neutral and does not own provider credentials or
source acquisition.

Terrarium height is decoded exactly from RGB bytes as
`256 * R + G + B / 256 - 32768` metres. An alpha byte of zero is NoData. Other
alpha values leave RGB unchanged. Heights below zero are preserved.

A mosaic has one zoom, an explicit inclusive XYZ source bound, and an explicit
RGBA payload memory limit. Tile input keys are validated and sorted by `(z,y,x)`
inside the immutable sampler. Bounds can wrap in X (`min_x > max_x`) across the
antimeridian; Y never wraps. Missing tiles and samples outside the supplied
bounds return NoData. No height is synthesized for uncovered ocean or empty
pixels.

`sample(latitude, longitude)` uses Web Mercator XYZ pixel-centre coordinates.
The latitude transform uses `g2m::det::tan` and `g2m::det::asinh`; longitude is
wrapped by whole turns. Bilinear interpolation reads the four required pixel
centres across tile boundaries, including the X wrap at the antimeridian. Any
required missing or alpha-zero pixel makes that interpolated sample NoData.
There is no edge extrapolation. Inputs must be finite, latitude must be within
[-90, 90], and longitude within [-540, 540]. The valid Web Mercator latitude
range is enforced; latitudes outside it return NoData.

`reproject(RasterGeoref, shift)` samples the supplied UTM lattice and returns an
`ElevTile` with the same registration and row order. It preserves NoData and
quantises metres to 1/256 m with half-to-even rounding. Output is limited to
4,194,304 samples; invalid georeferences, non-finite shifts, and heights outside
the encoded range raise `DecodeError`.

Tile size defaults to 512 and can be reduced for synthetic fixtures. Zoom is
limited to 0 through 30, and tile size to 1 through 4096. The memory limit
covers the owned RGBA payload. This API performs no tile fetching, image
unpacking, datum conversion, or blending between sources.
