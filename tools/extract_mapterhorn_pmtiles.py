#!/usr/bin/env python3
"""Extract a bounded local PMTiles v3 WebP selection to XYZ files.

This tool reads only a local archive and never performs network access.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import zlib

FORMAT = "g2m.mapterhorn_pmtiles_extract/1"
HEADER_SIZE = 127
MAX_ARCHIVE_BYTES = 64 * 1024 * 1024 * 1024
MAX_EXTRACT_TILES = 256
MAX_ROOT_BYTES = 16_257
MAX_DIRECTORY_BYTES = 8 * 1024 * 1024
MAX_DIRECTORY_ENTRIES = 100_000
MAX_TOTAL_LEAF_BYTES = 64 * 1024 * 1024
MAX_TOTAL_LEAF_DECOMPRESSED_BYTES = 32 * 1024 * 1024
MAX_TOTAL_LEAF_ENTRIES = 250_000
MAX_LEAF_DIRECTORY_BYTES = 8 * 1024 * 1024
MAX_TILE_BYTES = 8 * 1024 * 1024
MAX_LEAF_DEPTH = 4
MAX_VARINT = (1 << 64) - 1
WEB_MERCATOR_MAX_LAT = 85.0511287798066
EXTRACTION_LOCK = "pmtiles_extraction.json"
CLAIM_NAME = ".extract.lock"


class ArchiveError(ValueError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_exact(stream, length: int, what: str) -> bytes:
    data = stream.read(length)
    if len(data) != length:
        raise ArchiveError(f"truncated {what}")
    return data


def _u64(header: bytes, offset: int) -> int:
    return struct.unpack_from("<Q", header, offset)[0]


def _i32(header: bytes, offset: int) -> int:
    return struct.unpack_from("<i", header, offset)[0]


def parse_header(header: bytes, archive_size: int) -> dict:
    if len(header) != HEADER_SIZE:
        raise ArchiveError("PMTiles header must be exactly 127 bytes")
    if header[:7] != b"PMTiles":
        raise ArchiveError("not a PMTiles archive (bad magic)")
    if header[7] != 3:
        raise ArchiveError(f"PMTiles version {header[7]} is unsupported; expected v3")
    result = {
        "version": header[7],
        "root_offset": _u64(header, 8), "root_length": _u64(header, 16),
        "metadata_offset": _u64(header, 24), "metadata_length": _u64(header, 32),
        "leaf_offset": _u64(header, 40), "leaf_length": _u64(header, 48),
        "tile_offset": _u64(header, 56), "tile_length": _u64(header, 64),
        "addressed_tiles": _u64(header, 72), "tile_entries": _u64(header, 80),
        "tile_contents": _u64(header, 88), "clustered": header[96] == 1,
        "internal_compression": header[97], "tile_compression": header[98],
        "tile_type": header[99], "min_zoom": header[100], "max_zoom": header[101],
        "min_lon": _i32(header, 102) / 10_000_000,
        "min_lat": _i32(header, 106) / 10_000_000,
        "max_lon": _i32(header, 110) / 10_000_000,
        "max_lat": _i32(header, 114) / 10_000_000,
        "center_zoom": header[118], "center_lon": _i32(header, 119) / 10_000_000,
        "center_lat": _i32(header, 123) / 10_000_000,
    }
    if result["tile_type"] != 4:
        raise ArchiveError(f"PMTiles tile type {result['tile_type']} is not WebP (4)")
    if header[96] not in (0, 1):
        raise ArchiveError(f"invalid clustered flag {header[96]}")
    if result["internal_compression"] not in (1, 2):
        raise ArchiveError(f"unsupported internal compression {result['internal_compression']}; supports none/gzip")
    if result["tile_compression"] not in (1, 2):
        raise ArchiveError(f"unsupported tile compression {result['tile_compression']}; supports none/gzip")
    if result["min_zoom"] > result["max_zoom"]:
        raise ArchiveError("header minimum zoom exceeds maximum zoom")
    _check_section(result["root_offset"], result["root_length"], archive_size, "root directory")
    if result["root_offset"] < HEADER_SIZE or result["root_offset"] + result["root_length"] > 16_384:
        raise ArchiveError("root directory must be contained in archive bytes 127..16383")
    if result["root_length"] > MAX_ROOT_BYTES:
        raise ArchiveError("root directory exceeds PMTiles 16 KiB limit")
    _check_section(result["metadata_offset"], result["metadata_length"], archive_size, "metadata")
    if result["metadata_length"] == 0:
        raise ArchiveError("PMTiles metadata section is empty")
    _check_section(result["leaf_offset"], result["leaf_length"], archive_size, "leaf directories", optional=True)
    _check_section(result["tile_offset"], result["tile_length"], archive_size, "tile data", optional=True)
    sections = []
    for name, offset, length in (("root", result["root_offset"], result["root_length"]),
                                 ("metadata", result["metadata_offset"], result["metadata_length"]),
                                 ("leaf", result["leaf_offset"], result["leaf_length"]),
                                 ("tile data", result["tile_offset"], result["tile_length"])):
        if length:
            sections.append((offset, offset + length, name))
    sections.sort()
    for previous, current in zip(sections, sections[1:]):
        if current[0] < previous[1]:
            raise ArchiveError(f"PMTiles sections overlap: {previous[2]} and {current[2]}")
    return result


def _check_section(offset: int, length: int, archive_size: int, name: str, optional: bool = False) -> None:
    if length == 0:
        if optional:
            return
        raise ArchiveError(f"{name} section is empty")
    if offset < HEADER_SIZE:
        raise ArchiveError(f"{name} section overlaps the PMTiles header")
    if offset > archive_size or length > archive_size - offset:
        raise ArchiveError(f"{name} section extends beyond archive")


def decompress(data: bytes, compression: int, maximum: int, what: str) -> bytes:
    if compression == 1:
        if len(data) > maximum:
            raise ArchiveError(f"{what} exceeds {maximum} byte limit")
        return data
    if compression != 2:
        raise ArchiveError(f"unsupported compression {compression} for {what}")
    decoder = zlib.decompressobj(16 + zlib.MAX_WBITS)
    output = bytearray()
    try:
        output.extend(decoder.decompress(data, maximum + 1))
        while decoder.unconsumed_tail and len(output) <= maximum:
            output.extend(decoder.decompress(decoder.unconsumed_tail, maximum + 1 - len(output)))
    except zlib.error as error:
        raise ArchiveError(f"invalid gzip data for {what}: {error}") from error
    if len(output) > maximum:
        raise ArchiveError(f"decompressed {what} exceeds {maximum} byte limit")
    if not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ArchiveError(f"truncated or trailing gzip data for {what}")
    try:
        output.extend(decoder.flush(maximum + 1 - len(output)))
    except zlib.error as error:
        raise ArchiveError(f"invalid gzip trailer for {what}: {error}") from error
    if len(output) > maximum:
        raise ArchiveError(f"decompressed {what} exceeds {maximum} byte limit")
    return bytes(output)


def read_varint(data: bytes, position: int) -> tuple[int, int]:
    value = 0
    for index in range(10):
        if position >= len(data):
            raise ArchiveError("truncated directory varint")
        byte = data[position]
        position += 1
        payload = byte & 0x7F
        if index == 9 and payload > 1:
            raise ArchiveError("directory varint exceeds uint64")
        value |= payload << (7 * index)
        if byte < 0x80:
            if index > 0 and payload == 0:
                raise ArchiveError("non-canonical directory varint")
            return value, position
    raise ArchiveError("directory varint exceeds 10 bytes")


def decode_directory(raw: bytes, maximum_entries: int = MAX_DIRECTORY_ENTRIES) -> list[tuple[int, int, int, int]]:
    position = 0
    count, position = read_varint(raw, position)
    if not 0 < count <= min(MAX_DIRECTORY_ENTRIES, maximum_entries):
        raise ArchiveError(f"directory decoded-entry budget exceeded (count {count}, remaining {maximum_entries})")
    tile_ids = []
    last_id = 0
    for _ in range(count):
        delta, position = read_varint(raw, position)
        last_id += delta
        if last_id > MAX_VARINT:
            raise ArchiveError("directory TileID delta overflows uint64")
        if tile_ids and last_id <= tile_ids[-1]:
            raise ArchiveError("directory TileIDs are not strictly increasing")
        tile_ids.append(last_id)
    run_lengths = []
    for _ in range(count):
        value, position = read_varint(raw, position)
        run_lengths.append(value)
    lengths = []
    for _ in range(count):
        value, position = read_varint(raw, position)
        if value == 0:
            raise ArchiveError("directory entry length must be positive")
        lengths.append(value)
    offsets = []
    for index in range(count):
        value, position = read_varint(raw, position)
        if index and value == 0:
            offset = offsets[-1] + lengths[index - 1]
        else:
            if value == 0:
                raise ArchiveError("first directory offset cannot use the contiguous marker")
            offset = value - 1
        offsets.append(offset)
    for index, (tile_id, _, _, run_length) in enumerate(zip(tile_ids, offsets, lengths, run_lengths)):
        if run_length and index + 1 < count and tile_id + run_length > tile_ids[index + 1]:
            raise ArchiveError("directory tile runs overlap the next TileID")
    if position != len(raw):
        raise ArchiveError("directory has trailing bytes")
    return list(zip(tile_ids, offsets, lengths, run_lengths))


def rotate(n: int, x: int, y: int, rx: int, ry: int) -> tuple[int, int]:
    if ry == 0:
        if rx:
            x, y = n - 1 - x, n - 1 - y
        x, y = y, x
    return x, y


def zxy_to_tileid(z: int, x: int, y: int) -> int:
    if not 0 <= z <= 31 or not (0 <= x < 1 << z and 0 <= y < 1 << z):
        raise ArchiveError(f"invalid XYZ coordinate {z}/{x}/{y}")
    result = ((1 << (2 * z)) - 1) // 3
    a = z - 1
    while a >= 0:
        scale = 1 << a
        rx, ry = x & scale, y & scale
        result += ((3 * rx) ^ ry) << a
        x, y = rotate(scale, x, y, rx, ry)
        a -= 1
    return result


def tileid_to_zxy(tile_id: int) -> tuple[int, int, int]:
    if not 0 <= tile_id <= MAX_VARINT:
        raise ArchiveError("TileID outside uint64")
    z = ((3 * tile_id + 1).bit_length() - 1) // 2
    if z > 31:
        raise ArchiveError("TileID zoom exceeds PMTiles v3 uint64 range")
    acc = ((1 << (2 * z)) - 1) // 3
    position = tile_id - acc
    x = y = 0
    scale = 1
    while scale < 1 << z:
        rx = (position // 2) & scale
        ry = (position ^ rx) & scale
        x, y = rotate(scale, x, y, rx, ry)
        x += rx
        y += ry
        position >>= 1
        scale <<= 1
    return z, x, y


def bbox_xyz(bbox: tuple[float, float, float, float], zoom: int) -> list[tuple[int, int, int]]:
    west, south, east, north = bbox
    if not all(math.isfinite(value) for value in bbox):
        raise ArchiveError("bbox coordinates must be finite")
    if not (-180 <= west < east <= 180 and -WEB_MERCATOR_MAX_LAT <= south < north <= WEB_MERCATOR_MAX_LAT):
        raise ArchiveError("bbox must be ordered within Web Mercator bounds; split antimeridian selections")
    if not 0 <= zoom <= 31:
        raise ArchiveError("zoom must be in range 0..31")
    edge = 1 << zoom
    x0 = min(edge - 1, max(0, math.floor(edge * (west + 180.0) / 360.0)))
    x1 = min(edge, max(0, math.ceil(edge * (east + 180.0) / 360.0)))
    def row(latitude: float) -> float:
        radians = math.radians(latitude)
        return edge * (1.0 - math.asinh(math.tan(radians)) / math.pi) / 2.0
    y0 = min(edge - 1, max(0, math.floor(row(north))))
    y1 = min(edge, max(0, math.ceil(row(south))))
    count = (x1 - x0) * (y1 - y0)
    if count == 0:
        raise ArchiveError("bbox selects no XYZ tiles")
    if count > MAX_EXTRACT_TILES:
        raise ArchiveError(f"bbox selects {count} tiles; limit is {MAX_EXTRACT_TILES}")
    return [(zoom, x, y) for y in range(y0, y1) for x in range(x0, x1)]


class PMTilesArchive:
    def __init__(self, path: Path):
        self.path = path
        self.size = path.stat().st_size
        if not 0 < self.size <= MAX_ARCHIVE_BYTES:
            raise ArchiveError(f"archive size must be 1..{MAX_ARCHIVE_BYTES} bytes")
        self.initial_stat = path.stat()
        self.stream = path.open("rb")
        try:
            self.header = parse_header(_read_exact(self.stream, HEADER_SIZE, "PMTiles header"), self.size)
            if self.header["max_zoom"] > 31:
                raise ArchiveError("header zoom exceeds uint64 TileID range")
            self.leaf_bytes_read = 0
            self.leaf_decompressed_bytes = 0
            self.leaf_decoded_entries = 0
            self.leaf_cache: dict[tuple[int, int], list[tuple[int, int, int, int]]] = {}
            self.root, _ = self._directory(self.header["root_offset"], self.header["root_length"], "root")
            metadata_raw = self._section(self.header["metadata_offset"], self.header["metadata_length"], "metadata", MAX_DIRECTORY_BYTES)
            metadata_data = decompress(metadata_raw, self.header["internal_compression"], MAX_DIRECTORY_BYTES, "metadata")
            try:
                metadata = json.loads(metadata_data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise ArchiveError(f"invalid PMTiles JSON metadata: {error}") from error
            if not isinstance(metadata, dict):
                raise ArchiveError("PMTiles metadata must be a JSON object")
            self.metadata = metadata
        except Exception:
            self.stream.close()
            raise

    def close(self):
        self.stream.close()

    def _section(self, offset: int, length: int, name: str, maximum: int) -> bytes:
        if length > maximum:
            raise ArchiveError(f"{name} exceeds {maximum} byte limit")
        self.stream.seek(offset)
        return _read_exact(self.stream, length, name)

    def _directory(self, offset: int, length: int, name: str, *, max_decompressed: int = MAX_DIRECTORY_BYTES,
                   max_entries: int = MAX_DIRECTORY_ENTRIES) -> tuple[list[tuple[int, int, int, int]], int]:
        compressed_cap = MAX_ROOT_BYTES if name == "root" else MAX_LEAF_DIRECTORY_BYTES
        data = self._section(offset, length, name, compressed_cap)
        raw = decompress(data, self.header["internal_compression"], min(MAX_DIRECTORY_BYTES, max_decompressed), name)
        entries = decode_directory(raw, max_entries)
        for tile_id, entry_offset, entry_length, run_length in entries:
            bound = self.header["tile_length"]
            if run_length == 0:
                bound = self.header["leaf_length"]
            if entry_offset > bound or entry_length > bound - entry_offset:
                raise ArchiveError(f"{name} entry points outside its section")
            if name == "root" and run_length == 0 and not self.header["leaf_length"]:
                raise ArchiveError("root references a leaf directory but archive has no leaf section")
            if run_length and (tile_id + run_length - 1) > MAX_VARINT:
                raise ArchiveError(f"{name} tile run overflows uint64 TileID")
        return entries, len(raw)

    def _leaf(self, offset: int, length: int, depth: int) -> list[tuple[int, int, int, int]]:
        if depth > MAX_LEAF_DEPTH:
            raise ArchiveError(f"leaf directory recursion exceeds {MAX_LEAF_DEPTH}")
        identity = (offset, length)
        if identity in self.leaf_cache:
            return self.leaf_cache[identity]
        self.leaf_bytes_read += length
        if self.leaf_bytes_read > MAX_TOTAL_LEAF_BYTES:
            raise ArchiveError(f"leaf directory reads exceed {MAX_TOTAL_LEAF_BYTES} byte budget")
        remaining_bytes = MAX_TOTAL_LEAF_DECOMPRESSED_BYTES - self.leaf_decompressed_bytes
        remaining_entries = MAX_TOTAL_LEAF_ENTRIES - self.leaf_decoded_entries
        if remaining_bytes <= 0 or remaining_entries <= 0:
            raise ArchiveError("total leaf directory decoded-data budget exceeded")
        entries, decoded_bytes = self._directory(self.header["leaf_offset"] + offset, length, "leaf",
                                                 max_decompressed=remaining_bytes,
                                                 max_entries=remaining_entries)
        self.leaf_decompressed_bytes += decoded_bytes
        self.leaf_decoded_entries += len(entries)
        self.leaf_cache[identity] = entries
        return entries

    @staticmethod
    def _find(entries, tile_id):
        low, high = 0, len(entries) - 1
        while low <= high:
            middle = (low + high) // 2
            if tile_id > entries[middle][0]:
                low = middle + 1
            elif tile_id < entries[middle][0]:
                high = middle - 1
            else:
                entry = entries[middle]
                return entry if entry[3] == 0 or tile_id - entry[0] < entry[3] else None
        if high < 0:
            return None
        entry = entries[high]
        if entry[3] == 0 or tile_id - entry[0] < entry[3]:
            return entry
        return None

    def tile_entry(self, tile_id: int, depth: int = 0):
        entry = self._find(self.root, tile_id)
        if entry is None:
            return None
        if entry[3] != 0:
            return entry
        leaf = self._leaf(entry[1], entry[2], depth + 1)
        child = self._find(leaf, tile_id)
        if child is None:
            return None
        if child[3] == 0:
            return self.tile_entry_from_directory(child, tile_id, depth + 1)
        return child

    def tile_entry_from_directory(self, entry, tile_id, depth):
        if depth > MAX_LEAF_DEPTH:
            raise ArchiveError(f"leaf directory recursion exceeds {MAX_LEAF_DEPTH}")
        leaf = self._leaf(entry[1], entry[2], depth + 1)
        child = self._find(leaf, tile_id)
        if child is None or child[3] == 0:
            if child is None:
                return None
            return self.tile_entry_from_directory(child, tile_id, depth + 1)
        return child

    def read_tile(self, entry) -> bytes:
        _, offset, length, _ = entry
        if length > MAX_TILE_BYTES:
            raise ArchiveError(f"compressed tile blob exceeds {MAX_TILE_BYTES} byte limit")
        data = self._section(self.header["tile_offset"] + offset, length, "tile blob", MAX_TILE_BYTES)
        tile = decompress(data, self.header["tile_compression"], MAX_TILE_BYTES, "WebP tile")
        if len(tile) < 12 or tile[:4] != b"RIFF" or tile[8:12] != b"WEBP":
            raise ArchiveError("tile payload does not have a WebP RIFF signature")
        return tile

    def check_stable(self):
        current = self.path.stat()
        if (current.st_size, current.st_mtime_ns) != (self.initial_stat.st_size, self.initial_stat.st_mtime_ns):
            raise ArchiveError("archive changed while it was being extracted")


def _is_link(path: Path) -> bool:
    return path.is_symlink() or (hasattr(path, "is_junction") and path.is_junction())


def _reject_link_components(path: Path) -> None:
    absolute = Path(os.path.abspath(path))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current = current / part
        if _is_link(current):
            raise ArchiveError(f"output path contains a symbolic link or junction: {current}")


def _destination_root(destination: Path, *, create: bool) -> Path:
    root = Path(os.path.abspath(destination))
    _reject_link_components(root)
    if create:
        root.mkdir(parents=True, exist_ok=True)
    if not root.is_dir():
        raise ArchiveError(f"destination directory does not exist: {root}")
    _reject_link_components(root)
    return root.resolve(strict=True)


def _safe_output_path(root: Path, relative: str | Path) -> Path:
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ArchiveError(f"output path is not destination-relative: {relative}")
    candidate = root / relative
    _reject_link_components(candidate)
    root_real = root.resolve(strict=True)
    resolved = candidate.resolve(strict=False)
    try:
        resolved.relative_to(root_real)
    except ValueError as error:
        raise ArchiveError(f"output path escapes destination: {relative}") from error
    return candidate


def _ensure_output_parent(root: Path, relative: str | Path) -> None:
    current = root
    for part in Path(relative).parent.parts:
        if part in ("", "."):
            continue
        current = current / part
        _safe_output_path(root, current.relative_to(root))
        current.mkdir(exist_ok=True)
        if not current.is_dir():
            raise ArchiveError(f"output parent is not a directory: {current}")
        _safe_output_path(root, current.relative_to(root))


def _acquire_claim(root: Path) -> tuple[Path, tuple[int, int]]:
    claim = _safe_output_path(root, CLAIM_NAME)
    try:
        descriptor = os.open(str(claim), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as error:
        raise ArchiveError(f"another PMTiles extraction is active or left a stale claim at {claim}") from error
    identity = None
    try:
        stat = os.fstat(descriptor)
        identity = (stat.st_dev, stat.st_ino)
        os.write(descriptor, f"pid={os.getpid()}\n".encode("ascii"))
        os.fsync(descriptor)
        return claim, identity
    except Exception:
        os.close(descriptor)
        if identity is not None:
            _release_claim(root, claim, identity)
        raise
    finally:
        try:
            os.close(descriptor)
        except OSError:
            pass


def _release_claim(root: Path, claim: Path, identity: tuple[int, int]) -> None:
    try:
        _safe_output_path(root, claim.relative_to(root))
        stat = claim.lstat()
        if (stat.st_dev, stat.st_ino) == identity and not _is_link(claim):
            claim.unlink()
    except (FileNotFoundError, ArchiveError):
        pass


@contextmanager
def _destination_claim(root: Path):
    claim, identity = _acquire_claim(root)
    try:
        yield
    finally:
        _release_claim(root, claim, identity)


def _atomic_write_new(root: Path, relative: str | Path, data: bytes) -> Path:
    _ensure_output_parent(root, relative)
    path = _safe_output_path(root, relative)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=path.name + ".", suffix=".part", delete=False) as output:
            temporary = Path(output.name)
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        _safe_output_path(root, relative)
        if os.name == "nt":
            os.rename(temporary, path)  # Windows rename fails if path already exists.
        else:
            os.link(temporary, path)
        return path
    finally:
        if temporary is not None and temporary.exists():
            try:
                safe_temporary = _safe_output_path(root, temporary.relative_to(root))
                if not _is_link(safe_temporary):
                    safe_temporary.unlink()
            except (OSError, ArchiveError):
                pass


def extract(archive_path: Path, destination: Path, bbox: tuple[float, float, float, float], zoom: int) -> dict:
    targets = bbox_xyz(bbox, zoom)
    archive_path = archive_path.resolve()
    archive_stat = archive_path.stat()
    archive_size = archive_stat.st_size
    if not 0 < archive_size <= MAX_ARCHIVE_BYTES:
        raise ArchiveError(f"archive size must be 1..{MAX_ARCHIVE_BYTES} bytes")
    print(f"hashing full local archive ({archive_size} bytes) for the source pin")
    archive_hash = sha256_file(archive_path)
    after_hash_stat = archive_path.stat()
    if (after_hash_stat.st_size, after_hash_stat.st_mtime_ns) != (archive_stat.st_size, archive_stat.st_mtime_ns):
        raise ArchiveError("archive changed while its source hash was computed")
    reader = PMTilesArchive(archive_path)
    created: list[str] = []
    try:
        if (reader.initial_stat.st_size, reader.initial_stat.st_mtime_ns) != (archive_stat.st_size, archive_stat.st_mtime_ns):
            raise ArchiveError("archive changed between source hashing and parsing")
        if not reader.header["min_zoom"] <= zoom <= reader.header["max_zoom"]:
            raise ArchiveError(f"requested zoom {zoom} is outside archive zoom range {reader.header['min_zoom']}..{reader.header['max_zoom']}")
        tile_records = []
        total_uncompressed = 0
        for z, x, y in targets:
            tile_id = zxy_to_tileid(z, x, y)
            entry = reader.tile_entry(tile_id)
            if entry is None:
                continue
            tile = reader.read_tile(entry)
            total_uncompressed += len(tile)
            if total_uncompressed > MAX_EXTRACT_TILES * MAX_TILE_BYTES:
                raise ArchiveError("selected tile output exceeds total byte budget")
            relative = f"{z}/{x}/{y}.webp"
            tile_records.append({"z": z, "x": x, "y": y, "path": relative,
                                 "size": len(tile), "sha256": hashlib.sha256(tile).hexdigest(),
                                 "tile_id": tile_id, "blob_offset": entry[1], "blob_length": entry[2],
                                 "run_length": entry[3]})
        if not tile_records:
            raise ArchiveError("no archived tiles intersect the requested bbox and zoom; no elevation values were invented")
        header_summary = {key: reader.header[key] for key in (
            "version", "root_offset", "root_length", "metadata_offset", "metadata_length",
            "leaf_offset", "leaf_length", "tile_offset", "tile_length", "internal_compression",
            "tile_compression", "tile_type", "min_zoom", "max_zoom", "clustered")}
        manifest = {"format": FORMAT, "archive_name": archive_path.name, "archive_size": archive_size,
                    "archive_sha256": archive_hash, "bbox_wsen": list(bbox), "zoom": zoom,
                    "header": header_summary, "archive_metadata": reader.metadata,
                    "max_extract_tiles": MAX_EXTRACT_TILES, "max_tile_bytes": MAX_TILE_BYTES,
                    "requested_tile_count": len(targets), "extracted_tile_count": len(tile_records),
                    "missing_tile_count": len(targets) - len(tile_records), "tiles": tile_records}
        destination = Path(os.path.abspath(destination))
        _reject_link_components(destination)
        root = _destination_root(destination, create=True)
        with _destination_claim(root):
            try:
                lock_path = _safe_output_path(root, EXTRACTION_LOCK)
                prior = None
                if lock_path.exists():
                    try:
                        prior = json.loads(lock_path.read_text(encoding="utf-8"))
                    except (OSError, json.JSONDecodeError) as error:
                        raise ArchiveError("existing PMTiles extraction manifest is unreadable") from error
                    if not isinstance(prior, dict):
                        raise ArchiveError("existing PMTiles extraction manifest must be a JSON object")
                    for key in ("format", "archive_size", "archive_sha256", "bbox_wsen", "zoom"):
                        if prior.get(key) != manifest[key]:
                            raise ArchiveError(f"existing extraction pin differs at {key}; choose another destination")
                    if prior.get("tiles") != tile_records:
                        raise ArchiveError("existing extracted tile pins differ; choose another destination")
                for record in tile_records:
                    # Read again by pinned TileID; this keeps memory bounded to one WebP at a time.
                    entry = reader.tile_entry(record["tile_id"])
                    data = reader.read_tile(entry)
                    if hashlib.sha256(data).hexdigest() != record["sha256"]:
                        raise ArchiveError("archive tile bytes changed during extraction")
                    target = _safe_output_path(root, record["path"])
                    if target.resolve() == archive_path:
                        raise ArchiveError(f"output path would overwrite the archive: {record['path']}")
                    if target.exists():
                        if target.stat().st_size != len(data) or sha256_file(target) != record["sha256"]:
                            raise ArchiveError(f"refusing to overwrite changed output tile: {record['path']}")
                    else:
                        _atomic_write_new(root, record["path"], data)
                        created.append(record["path"])
                reader.check_stable()
                if lock_path.exists():
                    if prior is None:
                        raise ArchiveError("extraction manifest appeared during extraction; refusing to overwrite it")
                    return prior
                _atomic_write_new(root, EXTRACTION_LOCK,
                                  (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode("utf-8"))
                return manifest
            except Exception:
                for relative in created:
                    try:
                        path = _safe_output_path(root, relative)
                        if path.exists() and not _is_link(path):
                            path.unlink()
                    except (FileNotFoundError, ArchiveError):
                        pass
                raise
    finally:
        reader.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True, help="local PMTiles v3 archive")
    parser.add_argument("--destination", type=Path, required=True, help="directory for z/x/y.webp tiles")
    parser.add_argument("--bbox", nargs=4, type=float, required=True, metavar=("WEST", "SOUTH", "EAST", "NORTH"))
    parser.add_argument("--zoom", type=int, required=True, help="one XYZ zoom level")
    args = parser.parse_args(argv)
    try:
        manifest = extract(args.archive, args.destination, tuple(args.bbox), args.zoom)
        print(f"extracted {manifest['extracted_tile_count']} of {manifest['requested_tile_count']} intersecting tiles")
        if manifest["missing_tile_count"]:
            print(f"{manifest['missing_tile_count']} requested tiles are absent from the archive")
        attribution = manifest["archive_metadata"].get("attribution")
        if not attribution:
            print("archive metadata has no attribution field; dataset licensing remains caller responsibility")
        return 0
    except (OSError, ArchiveError) as error:
        parser.error(str(error))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
