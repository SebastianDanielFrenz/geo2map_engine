#!/usr/bin/env python3
"""Prepare an offline Mapterhorn WebP XYZ extract as pinned RGBA8 tiles.

This tool never accesses the network. Pillow is imported only when a new
extract is decoded; --verify checks the lock and raw files using stdlib only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Callable

FORMAT = "g2m.mapterhorn_rgba/1"
LOCK_NAME = "sources.json"
TILE_SIZE = 512
MAX_ZOOM = 24
MAX_TILES = 4096
MAX_SOURCE_FILE_BYTES = 8 * 1024 * 1024
MAX_TOTAL_SOURCE_BYTES = 2 * 1024 * 1024 * 1024
MAX_RAW_FILE_BYTES = TILE_SIZE * TILE_SIZE * 4
MAX_TOTAL_RAW_BYTES = MAX_TILES * MAX_RAW_FILE_BYTES
CLAIM_NAME = ".prepare.lock"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_source_bytes(path: Path) -> bytes:
    with path.open("rb") as source:
        data = source.read(MAX_SOURCE_FILE_BYTES + 1)
    if not 0 < len(data) <= MAX_SOURCE_FILE_BYTES:
        raise ValueError(f"source tile exceeds {MAX_SOURCE_FILE_BYTES} byte limit: {path}")
    return data


def tile_key(relative: Path) -> tuple[int, int, int]:
    """Validate the exact z/x/y.webp layout and XYZ bounds."""
    if len(relative.parts) != 3 or relative.suffix.lower() != ".webp":
        raise ValueError(f"tile must use z/x/y.webp layout: {relative}")
    try:
        z, x = int(relative.parts[0]), int(relative.parts[1])
        y = int(relative.stem)
    except ValueError as error:
        raise ValueError(f"non-integer XYZ tile path: {relative}") from error
    if str(z) != relative.parts[0] or str(x) != relative.parts[1] or str(y) != relative.stem:
        raise ValueError(f"non-canonical XYZ tile path: {relative}")
    if not 0 <= z <= MAX_ZOOM:
        raise ValueError(f"zoom must be between 0 and {MAX_ZOOM}: {relative}")
    edge = 1 << z
    if not (0 <= x < edge and 0 <= y < edge):
        raise ValueError(f"XYZ coordinate outside zoom {z}: {relative}")
    return z, x, y


def discover(source: Path, limitzoom: int | None) -> list[tuple[Path, tuple[int, int, int]]]:
    if not source.is_dir():
        raise ValueError(f"source directory does not exist: {source}")
    if limitzoom is not None and not 0 <= limitzoom <= MAX_ZOOM:
        raise ValueError(f"--limitzoom must be between 0 and {MAX_ZOOM}")
    root = source.resolve()
    tiles: list[tuple[Path, tuple[int, int, int]]] = []
    total_bytes = 0
    for path in source.rglob("*.webp"):
        if path.is_symlink():
            raise ValueError(f"symbolic-link tiles are not accepted: {path}")
        resolved = path.resolve()
        if not resolved.is_relative_to(root):
            raise ValueError(f"tile path escapes source directory: {path}")
        relative = path.relative_to(source)
        key = tile_key(relative)
        if limitzoom is not None and key[0] > limitzoom:
            continue
        size = path.stat().st_size
        if not 0 < size <= MAX_SOURCE_FILE_BYTES:
            raise ValueError(f"source tile size must be 1..{MAX_SOURCE_FILE_BYTES} bytes: {relative}")
        total_bytes += size
        tiles.append((path, key))
        if len(tiles) > MAX_TILES:
            raise ValueError(f"tile count exceeds maximum {MAX_TILES}")
        if total_bytes > MAX_TOTAL_SOURCE_BYTES:
            raise ValueError(f"selected source bytes exceed maximum {MAX_TOTAL_SOURCE_BYTES}")
    tiles.sort(key=lambda item: item[1])
    if not tiles:
        raise ValueError("no WebP source tiles found at z/x/y.webp (check --limitzoom)")
    return tiles


def read_metadata(path: Path | None) -> dict:
    if path is None:
        return {"vertical_reference": "unknown", "attribution": None, "dataset_license": None}
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("metadata JSON must be an object")
    # Preserve caller-provided provenance without assigning a dataset-wide licence.
    return value


def decode_webp(data: bytes) -> tuple[bytes, int, int]:
    """Decode a bounded WebP byte string to tightly packed row-major RGBA8."""
    try:
        from PIL import Image
    except ImportError as error:
        raise RuntimeError("Pillow is required to prepare WebP tiles; install it in the chosen Python environment") from error
    import io
    import warnings

    with warnings.catch_warnings():
        warnings.simplefilter("error", Image.DecompressionBombWarning)
        with Image.open(io.BytesIO(data)) as image:
            width, height = image.size
            if width != TILE_SIZE or height != TILE_SIZE:
                raise ValueError(f"expected {TILE_SIZE}x{TILE_SIZE} WebP tile, got {width}x{height}")
            if image.format != "WEBP":
                raise ValueError(f"expected WebP image, got {image.format}")
            if getattr(image, "n_frames", 1) != 1:
                raise ValueError("animated WebP tiles are not supported")
            image.load()
            rgba = image.convert("RGBA").tobytes()
    if len(rgba) != width * height * 4:
        raise ValueError("decoder returned an unexpected RGBA byte count")
    return rgba, width, height


def _is_link(path: Path) -> bool:
    return path.is_symlink() or (hasattr(path, "is_junction") and path.is_junction())


def _reject_link_components(path: Path) -> None:
    absolute = Path(os.path.abspath(path))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current = current / part
        if _is_link(current):
            raise ValueError(f"output path contains a symbolic link or junction: {current}")


def _destination_root(destination: Path, *, create: bool) -> Path:
    root = Path(os.path.abspath(destination))
    _reject_link_components(root)
    if create:
        root.mkdir(parents=True, exist_ok=True)
    if not root.is_dir():
        raise ValueError(f"destination directory does not exist: {root}")
    _reject_link_components(root)
    return root.resolve(strict=True)


def _safe_output_path(root: Path, relative: str | Path) -> Path:
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"output path is not destination-relative: {relative}")
    candidate = root / relative
    _reject_link_components(candidate)
    root_real = root.resolve(strict=True)
    resolved = candidate.resolve(strict=False)
    try:
        resolved.relative_to(root_real)
    except ValueError as error:
        raise ValueError(f"output path escapes destination: {relative}") from error
    return candidate


def _ensure_output_parent(root: Path, relative: str | Path) -> None:
    parent = Path(relative).parent
    current = root
    for part in parent.parts:
        if part in ("", "."):
            continue
        current = current / part
        _safe_output_path(root, current.relative_to(root))
        current.mkdir(exist_ok=True)
        if not current.is_dir():
            raise ValueError(f"output parent is not a directory: {current}")
        _safe_output_path(root, current.relative_to(root))


def _acquire_claim(root: Path) -> tuple[Path, tuple[int, int]]:
    claim = _safe_output_path(root, CLAIM_NAME)
    try:
        descriptor = os.open(str(claim), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as error:
        raise ValueError(f"another preparation is active or left a stale claim at {claim}") from error
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
    except (FileNotFoundError, ValueError):
        pass


def _atomic_write_new(root: Path, relative: str | Path, data: bytes) -> Path:
    _ensure_output_parent(root, relative)
    path = _safe_output_path(root, relative)
    temporary: Path | None = None
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
            except (OSError, ValueError):
                pass


def _load_lock(lock_path: Path) -> dict:
    try:
        manifest = json.loads(lock_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read source lock: {lock_path}") from error
    if not isinstance(manifest, dict) or manifest.get("format") != FORMAT:
        raise ValueError("unknown Mapterhorn source lock format")
    if not isinstance(manifest.get("tiles"), list) or not manifest["tiles"]:
        raise ValueError("source lock has no tiles")
    return manifest


def _source_records(source: Path, selected: list[tuple[Path, tuple[int, int, int]]]) -> list[dict]:
    records = []
    for path, (z, x, y) in selected:
        size = path.stat().st_size
        if not 0 < size <= MAX_SOURCE_FILE_BYTES:
            raise ValueError(f"source tile size must be 1..{MAX_SOURCE_FILE_BYTES} bytes: {path}")
        records.append({"z": z, "x": x, "y": y, "path": path.relative_to(source).as_posix(),
                        "source_bytes": size, "source_sha256": sha256_file(path)})
    return records


def _verify(source: Path, destination: Path, limitzoom: int | None = None) -> dict:
    root = _destination_root(destination, create=False)
    lock_path = _safe_output_path(root, LOCK_NAME)
    manifest = _load_lock(lock_path)
    if manifest.get("tile_size") != TILE_SIZE or manifest.get("raw_format") != "rgba8-row-major":
        raise ValueError("unsupported raw tile dimensions or encoding in source lock")
    locked = manifest["tiles"]
    if len(locked) > MAX_TILES:
        raise ValueError("source lock exceeds tile-count budget")
    pinned_zoom = manifest.get("limitzoom")
    if limitzoom is not None and limitzoom != pinned_zoom:
        raise ValueError("--limitzoom differs from sources.json; use a new destination for a different extract")
    selected = discover(source, pinned_zoom)
    current = _source_records(source, selected)
    expected = [(entry.get("z"), entry.get("x"), entry.get("y"), entry.get("input_path")) for entry in locked]
    actual = [(entry["z"], entry["x"], entry["y"], entry["path"]) for entry in current]
    if actual != expected:
        raise ValueError("source tile set differs from sources.json; use a new destination for a different extract")
    raw_total = 0
    for entry, present in zip(locked, current):
        if entry.get("input_size") != present["source_bytes"] or entry.get("input_sha256") != present["source_sha256"]:
            raise ValueError(f"source tile changed: {present['path']}")
        expected_raw_path = f"{present['z']}/{present['x']}/{present['y']}.rgba"
        if entry.get("path") != expected_raw_path:
            raise ValueError(f"invalid raw tile path in source lock: {present['path']}")
        raw_path = _safe_output_path(root, expected_raw_path)
        raw_bytes = entry.get("size")
        raw_hash = entry.get("sha256")
        if not isinstance(raw_bytes, int) or not 0 < raw_bytes <= MAX_RAW_FILE_BYTES or not isinstance(raw_hash, str) or not _SHA256.fullmatch(raw_hash):
            raise ValueError(f"invalid raw tile pin: {present['path']}")
        raw_total += raw_bytes
        if raw_total > MAX_TOTAL_RAW_BYTES:
            raise ValueError("raw output exceeds byte budget")
        if not raw_path.is_file() or raw_path.stat().st_size != raw_bytes or sha256_file(raw_path) != raw_hash:
            raise ValueError(f"raw output missing or changed: {entry.get('path')}")
    return manifest


def prepare(source: Path, destination: Path, *, limitzoom: int | None = None,
            metadata_path: Path | None = None,
            decoder: Callable[[bytes], tuple[bytes, int, int]] | None = None) -> dict:
    """Decode source tiles and pin both compressed input and decoded pixels.

    `decoder` is injectable for tests; production defaults to Pillow lazily.
    """
    destination = Path(os.path.abspath(destination))
    _reject_link_components(destination)
    root = _destination_root(destination, create=False) if destination.exists() else None
    lock_path = _safe_output_path(root, LOCK_NAME) if root is not None else destination / LOCK_NAME
    if lock_path.exists():
        manifest = _load_lock(lock_path)
        # Verify first so a changed source never gets silently accepted.
        _verify(source, destination, limitzoom)
        if metadata_path is not None and read_metadata(metadata_path) != manifest.get("source_metadata"):
            raise ValueError("caller metadata differs from the pinned lock; use a new destination")
        return manifest

    selected = discover(source, limitzoom)
    records = _source_records(source, selected)
    metadata = read_metadata(metadata_path)
    if metadata_path is None:
        print("warning: no caller metadata supplied; vertical datum and attribution are unknown")
    total_raw = len(records) * MAX_RAW_FILE_BYTES
    if total_raw > MAX_TOTAL_RAW_BYTES:
        raise ValueError("raw output exceeds byte budget")
    decode = decoder or decode_webp
    root = _destination_root(destination, create=True)
    claim, claim_identity = _acquire_claim(root)
    tiles = []
    written: list[str] = []
    try:
        lock_path = _safe_output_path(root, LOCK_NAME)
        if lock_path.exists():
            manifest = _load_lock(lock_path)
            _verify(source, root, limitzoom)
            if metadata_path is not None and read_metadata(metadata_path) != manifest.get("source_metadata"):
                raise ValueError("caller metadata differs from the pinned lock; use a new destination")
            return manifest
        for record, (source_path, _) in zip(records, selected):
            data = read_source_bytes(source_path)
            if len(data) != record["source_bytes"] or sha256_bytes(data) != record["source_sha256"]:
                raise ValueError(f"source tile changed while reading: {record['path']}")
            rgba, width, height = decode(data)
            if width != TILE_SIZE or height != TILE_SIZE:
                raise ValueError(f"expected {TILE_SIZE}x{TILE_SIZE} decoded tile, got {width}x{height}: {record['path']}")
            raw_bytes = width * height * 4
            if len(rgba) != raw_bytes or raw_bytes > MAX_RAW_FILE_BYTES:
                raise ValueError(f"decoder returned invalid/budget-exceeding RGBA data: {record['path']}")
            raw_path = f"{record['z']}/{record['x']}/{record['y']}.rgba"
            target = _safe_output_path(root, raw_path)
            if target.exists():
                raise ValueError(f"refusing to overwrite unpinned raw output: {raw_path}")
            _atomic_write_new(root, raw_path, rgba)
            written.append(raw_path)
            tiles.append({"z": record["z"], "x": record["x"], "y": record["y"],
                          "width": width, "height": height, "path": raw_path,
                          "sha256": sha256_bytes(rgba), "size": raw_bytes,
                          "input_path": record["path"], "input_sha256": record["source_sha256"],
                          "input_size": record["source_bytes"]})
        manifest = {"format": FORMAT, "tile_size": TILE_SIZE, "raw_format": "rgba8-row-major",
                    "tile_count": len(tiles), "limitzoom": limitzoom,
                    "max_tile_count": MAX_TILES, "max_source_file_bytes": MAX_SOURCE_FILE_BYTES,
                    "max_raw_file_bytes": MAX_RAW_FILE_BYTES, "source_metadata": metadata,
                    "tiles": tiles}
        _atomic_write_new(root, LOCK_NAME, (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode("utf-8"))
        return manifest
    except Exception:
        for relative in written:
            try:
                path = _safe_output_path(root, relative)
                if path.exists() and not _is_link(path):
                    path.unlink()
            except (FileNotFoundError, ValueError):
                pass
        raise
    finally:
        _release_claim(root, claim, claim_identity)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="local/extracted directory containing z/x/y.webp")
    parser.add_argument("--destination", type=Path, required=True, help="prepared raw-tile directory")
    parser.add_argument("--limitzoom", type=int, help=f"only prepare zooms <= Z (0..{MAX_ZOOM})")
    parser.add_argument("--metadata", type=Path, help="caller JSON metadata (attribution, dataset_license, vertical_reference, etc.)")
    parser.add_argument("--verify", action="store_true", help="offline verify pinned source and RGBA hashes; Pillow is not needed")
    args = parser.parse_args(argv)
    try:
        if args.verify:
            manifest = _verify(args.source, args.destination, args.limitzoom)
            print(f"verified {len(manifest['tiles'])} pinned Mapterhorn RGBA tiles offline")
        else:
            manifest = prepare(args.source, args.destination, limitzoom=args.limitzoom, metadata_path=args.metadata)
            print(f"prepared {len(manifest['tiles'])} Mapterhorn RGBA tiles at {args.destination}")
        missing = []
        metadata = manifest.get("source_metadata", {})
        for field in ("attribution", "dataset_license", "vertical_reference"):
            if not metadata.get(field):
                missing.append(field)
        if missing:
            print("metadata missing/unknown: " + ", ".join(missing))
        return 0
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
        parser.error(str(error))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
