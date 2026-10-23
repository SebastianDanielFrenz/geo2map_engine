import gzip
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import struct
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).with_name("extract_mapterhorn_pmtiles.py")
SPEC = importlib.util.spec_from_file_location("extract_mapterhorn_pmtiles", SCRIPT)
pm = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pm)


def varint(value):
    out = bytearray()
    while value > 0x7F:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def directory(entries):
    """Entries are (tile_id, offset, length, run_length)."""
    out = bytearray(varint(len(entries)))
    previous_id = 0
    for tile_id, _, _, _ in entries:
        out.extend(varint(tile_id - previous_id))
        previous_id = tile_id
    for _, _, _, run_length in entries:
        out.extend(varint(run_length))
    for _, _, length, _ in entries:
        out.extend(varint(length))
    for index, (_, offset, length, _) in enumerate(entries):
        if index and offset == entries[index - 1][1] + entries[index - 1][2]:
            out.extend(varint(0))
        else:
            out.extend(varint(offset + 1))
    return bytes(out)


def webp(marker):
    payload = bytes((marker, marker ^ 0x55))
    body = b"WEBP" + payload
    return b"RIFF" + struct.pack("<I", len(body)) + body


def make_archive(path, *, leaf=False, internal_gzip=False, tile_gzip=False, tile_type=4, internal_compression=None,
                 selected=(0, 1, 2, 3), tile_compression=None):
    all_tiles = [(1, webp(1)), (2, webp(2)), (3, webp(3)), (4, webp(4))]
    tiles = [all_tiles[index] for index in selected]
    blobs = [gzip.compress(data, mtime=0) if tile_gzip else data for _, data in tiles]
    tile_data = b"".join(blobs)
    offsets = []
    pos = 0
    for blob in blobs:
        offsets.append(pos)
        pos += len(blob)
    compress = gzip.compress if internal_gzip else (lambda data, mtime=0: data)
    ic = internal_compression if internal_compression is not None else (2 if internal_gzip else 1)
    metadata = compress(json.dumps({"name": "synthetic", "attribution": "fixture credit"}).encode(), mtime=0)
    if leaf:
        if len(tiles) != 4:
            raise ValueError("leaf fixture currently requires all four tiles")
        leaf_raw = directory([(tiles[i][0], offsets[i], len(blobs[i]), 1) for i in range(1, 4)])
        leaf_bytes = compress(leaf_raw, mtime=0)
        root_raw = directory([(tiles[0][0], offsets[0], len(blobs[0]), 1), (tiles[1][0], 0, len(leaf_bytes), 0)])
        root_bytes = compress(root_raw, mtime=0)
    else:
        leaf_bytes = b""
        root_raw = directory([(tiles[i][0], offsets[i], len(blobs[i]), 1) for i in range(len(tiles))])
        root_bytes = compress(root_raw, mtime=0)
    root_off = 127
    metadata_off = root_off + len(root_bytes)
    leaf_off = metadata_off + len(metadata)
    tile_off = leaf_off + len(leaf_bytes)
    header = bytearray(127)
    header[0:7] = b"PMTiles"
    header[7] = 3
    for offset, value in ((8, root_off), (16, len(root_bytes)), (24, metadata_off), (32, len(metadata)),
                          (40, leaf_off), (48, len(leaf_bytes)), (56, tile_off), (64, len(tile_data)),
                          (72, len(tiles)), (80, len(tiles)), (88, len(tiles))):
        struct.pack_into("<Q", header, offset, value)
    header[96] = 1
    header[97] = ic
    header[98] = tile_compression if tile_compression is not None else (2 if tile_gzip else 1)
    header[99] = tile_type
    header[100] = 1
    header[101] = 1
    for offset, value in ((102, -180_0000000), (106, -85_0000000), (110, 180_0000000), (114, 85_0000000),
                          (119, 0), (123, 0)):
        struct.pack_into("<i", header, offset, value)
    header[118] = 1
    path.write_bytes(bytes(header) + root_bytes + metadata + leaf_bytes + tile_data)
    all_expected = {(1, 0, 0): webp(1), (1, 0, 1): webp(2), (1, 1, 1): webp(3), (1, 1, 0): webp(4)}
    return {"tiles": {coords: all_expected[coords] for coords in all_expected}}


class ExtractMapterhornPMTilesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=SCRIPT.parent)
        self.root = Path(self.temp.name)
        self.archive = self.root / "fixture.pmtiles"
        self.output = self.root / "tiles"

    def tearDown(self):
        self.temp.cleanup()

    def symlink_dir(self, link, target):
        try:
            os.symlink(target, link, target_is_directory=True)
        except (OSError, NotImplementedError):
            self.skipTest("directory symlinks are unavailable on this platform")

    def fake_leaf_reader(self):
        raw = directory([(1, 0, 1, 1), (2, 0, 1, 1)])
        compressed = gzip.compress(raw, mtime=0)
        reader = pm.PMTilesArchive.__new__(pm.PMTilesArchive)
        reader.stream = io.BytesIO(compressed + compressed)
        reader.header = {"internal_compression": 2, "leaf_offset": 0, "leaf_length": 2 * len(compressed), "tile_length": 1}
        reader.leaf_bytes_read = 0
        reader.leaf_decompressed_bytes = 0
        reader.leaf_decoded_entries = 0
        reader.leaf_cache = {}
        return reader, len(compressed), len(raw)

    def test_header_and_directory_fixture_with_leaf_and_gzip(self):
        expected = make_archive(self.archive, leaf=True, internal_gzip=True, tile_gzip=True)
        manifest = pm.extract(self.archive, self.output, (-180.0, -85.0, 0.0, 85.0), 1)
        self.assertEqual(manifest["extracted_tile_count"], 2)
        self.assertEqual(manifest["missing_tile_count"], 0)
        for coords in ((1, 0, 0), (1, 0, 1)):
            raw = self.output / str(coords[0]) / str(coords[1]) / f"{coords[2]}.webp"
            self.assertEqual(raw.read_bytes(), expected["tiles"][coords])
        self.assertEqual(manifest["archive_sha256"], hashlib.sha256(self.archive.read_bytes()).hexdigest())
        self.assertEqual(pm.extract(self.archive, self.output, (-180.0, -85.0, 0.0, 85.0), 1), manifest)

    def test_xyz_clipping_and_known_hilbert_ids(self):
        self.assertEqual(pm.bbox_xyz((-180.0, -85.0, 0.0, 85.0), 1), [(1, 0, 0), (1, 0, 1)])
        self.assertEqual([pm.zxy_to_tileid(1, x, y) for x, y in ((0, 0), (0, 1), (1, 1), (1, 0))], [1, 2, 3, 4])
        self.assertEqual(pm.zxy_to_tileid(2, 0, 0), 5)
        for tile_id in range(21):
            z, x, y = pm.tileid_to_zxy(tile_id)
            self.assertEqual(pm.zxy_to_tileid(z, x, y), tile_id)
        self.assertEqual(pm.bbox_xyz((-180, -85, 0, 0), 1), [(1, 0, 1)])

    def test_missing_tiles_are_reported_and_not_filled(self):
        make_archive(self.archive, selected=(0, 2, 3))
        manifest = pm.extract(self.archive, self.output, (-180, -85, 180, 85), 1)
        self.assertEqual(manifest["requested_tile_count"], 4)
        self.assertEqual(manifest["extracted_tile_count"], 3)
        self.assertEqual(manifest["missing_tile_count"], 1)
        self.assertFalse((self.output / "1" / "0" / "1.webp").exists())

    def test_corrupt_directory_varint_and_archive_are_rejected(self):
        with self.assertRaisesRegex(pm.ArchiveError, "truncated directory varint"):
            pm.read_varint(b"\x80", 0)
        with self.assertRaisesRegex(pm.ArchiveError, "uint64"):
            pm.read_varint(b"\xff" * 9 + b"\x02", 0)
        self.archive.write_bytes(b"not pmtiles")
        with self.assertRaisesRegex(pm.ArchiveError, "truncated PMTiles header"):
            pm.PMTilesArchive(self.archive)

    def test_unsupported_compressions_and_tile_type_are_rejected(self):
        make_archive(self.archive, internal_compression=3)
        with self.assertRaisesRegex(pm.ArchiveError, "internal compression 3"):
            pm.PMTilesArchive(self.archive)
        make_archive(self.archive, tile_compression=4)
        with self.assertRaisesRegex(pm.ArchiveError, "tile compression 4"):
            pm.PMTilesArchive(self.archive)
        make_archive(self.archive, tile_type=2)
        with self.assertRaisesRegex(pm.ArchiveError, "not WebP"):
            pm.PMTilesArchive(self.archive)

    def test_output_pin_refuses_changed_file(self):
        make_archive(self.archive)
        pm.extract(self.archive, self.output, (-180, -85, 0, 85), 1)
        output_tile = self.output / "1" / "0" / "0.webp"
        output_tile.write_bytes(b"changed")
        with self.assertRaisesRegex(pm.ArchiveError, "refusing to overwrite changed output"):
            pm.extract(self.archive, self.output, (-180, -85, 0, 85), 1)

    def test_bad_root_offset_is_rejected(self):
        make_archive(self.archive)
        raw = bytearray(self.archive.read_bytes())
        struct.pack_into("<Q", raw, 8, len(raw) + 1)
        self.archive.write_bytes(raw)
        with self.assertRaisesRegex(pm.ArchiveError, "root directory section extends beyond archive"):
            pm.PMTilesArchive(self.archive)

    def test_existing_claim_blocks_mutation_and_is_preserved(self):
        make_archive(self.archive)
        self.output.mkdir()
        claim = self.output / pm.CLAIM_NAME
        claim.write_text("pid=winner\n", encoding="utf-8")
        with self.assertRaisesRegex(pm.ArchiveError, "another PMTiles extraction is active"):
            pm.extract(self.archive, self.output, (-180, -85, 0, 85), 1)
        self.assertEqual(claim.read_text(encoding="utf-8"), "pid=winner\n")
        self.assertFalse((self.output / "1" / "0" / "0.webp").exists())

    def test_symlinked_output_parent_is_rejected_before_write(self):
        make_archive(self.archive)
        self.output.mkdir()
        outside = self.root / "outside"
        outside.mkdir()
        self.symlink_dir(self.output / "1", outside)
        with self.assertRaisesRegex(pm.ArchiveError, "symbolic link or junction"):
            pm.extract(self.archive, self.output, (-180, -85, 0, 85), 1)
        self.assertEqual(list(outside.iterdir()), [])
        self.assertFalse((self.output / pm.CLAIM_NAME).exists())

    def test_leaf_cache_total_decompressed_bytes_are_bounded(self):
        reader, compressed_length, raw_length = self.fake_leaf_reader()
        with patch.object(pm, "MAX_TOTAL_LEAF_DECOMPRESSED_BYTES", raw_length * 2 - 1):
            self.assertEqual(len(reader._leaf(0, compressed_length, 1)), 2)
            with self.assertRaisesRegex(pm.ArchiveError, "decompressed leaf exceeds"):
                reader._leaf(compressed_length, compressed_length, 1)
        reader.stream.close()

    def test_leaf_cache_total_entry_objects_are_bounded_before_decode(self):
        reader, compressed_length, _ = self.fake_leaf_reader()
        with patch.object(pm, "MAX_TOTAL_LEAF_ENTRIES", 3):
            self.assertEqual(len(reader._leaf(0, compressed_length, 1)), 2)
            with self.assertRaisesRegex(pm.ArchiveError, "decoded-entry budget exceeded"):
                reader._leaf(compressed_length, compressed_length, 1)
        reader.stream.close()


if __name__ == "__main__":
    unittest.main()
