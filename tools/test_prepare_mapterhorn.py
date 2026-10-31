import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).with_name("prepare_mapterhorn.py")
SPEC = importlib.util.spec_from_file_location("prepare_mapterhorn", SCRIPT)
prepare_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(prepare_module)


def fake_decode(_data):
    return bytes((17, 34, 51, 255)) * (512 * 512), 512, 512


class PrepareMapterhornTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=SCRIPT.parent)
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.output = self.root / "prepared"

    def tearDown(self):
        self.temp.cleanup()

    def tile(self, z=1, x=0, y=0, data=b"test webp payload"):
        path = self.source / str(z) / str(x) / f"{y}.webp"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return path

    def symlink_dir(self, link, target):
        try:
            os.symlink(target, link, target_is_directory=True)
        except (OSError, NotImplementedError):
            self.skipTest("directory symlinks are unavailable on this platform")

    def test_existing_claim_blocks_mutation_and_is_preserved(self):
        self.tile()
        self.output.mkdir()
        claim = self.output / prepare_module.CLAIM_NAME
        claim.write_text("pid=winner\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "another preparation is active"):
            prepare_module.prepare(self.source, self.output, decoder=fake_decode)
        self.assertEqual(claim.read_text(encoding="utf-8"), "pid=winner\n")
        self.assertFalse((self.output / "1" / "0" / "0.rgba").exists())

    def test_symlinked_output_parent_is_rejected_before_write(self):
        self.tile()
        self.output.mkdir()
        outside = self.root / "outside"
        outside.mkdir()
        self.symlink_dir(self.output / "1", outside)
        with self.assertRaisesRegex(ValueError, "symbolic link or junction"):
            prepare_module.prepare(self.source, self.output, decoder=fake_decode)
        self.assertEqual(list(outside.iterdir()), [])
        self.assertFalse((self.output / prepare_module.CLAIM_NAME).exists())

    def test_verify_rejects_symlinked_output_parent(self):
        self.tile()
        manifest = prepare_module.prepare(self.source, self.output, decoder=fake_decode)
        outside = self.root / "outside"
        outside.mkdir()
        shutil.rmtree(self.output / "1")
        self.symlink_dir(self.output / "1", outside)
        with self.assertRaisesRegex(ValueError, "symbolic link or junction"):
            prepare_module._verify(self.source, self.output)
        self.assertEqual(manifest["tile_count"], 1)

    def test_prepares_contract_and_offline_verifies_without_decoder(self):
        compressed = b"opaque test fixture, not a WebP image"
        input_path = self.tile(data=compressed)
        metadata_path = self.root / "metadata.json"
        metadata_path.write_text(json.dumps({"vertical_reference": "unknown", "attribution": "Caller credit"}), encoding="utf-8")

        manifest = prepare_module.prepare(self.source, self.output, metadata_path=metadata_path, decoder=fake_decode)
        tile = manifest["tiles"][0]
        self.assertEqual(set(("z", "x", "y", "width", "height", "path", "sha256", "size", "input_path", "input_sha256", "input_size")) - set(tile), set())
        self.assertEqual((tile["z"], tile["x"], tile["y"], tile["width"], tile["height"]), (1, 0, 0, 512, 512))
        self.assertEqual(tile["path"], "1/0/0.rgba")
        self.assertEqual(tile["input_path"], "1/0/0.webp")
        self.assertEqual(tile["input_sha256"], hashlib.sha256(compressed).hexdigest())
        self.assertEqual(tile["sha256"], hashlib.sha256(fake_decode(b"")[0]).hexdigest())
        self.assertEqual(tile["size"], 512 * 512 * 4)
        self.assertEqual((self.output / tile["path"]).read_bytes()[:4], bytes((17, 34, 51, 255)))
        with patch.object(prepare_module, "decode_webp", side_effect=AssertionError("verify must not decode")):
            verified = prepare_module._verify(self.source, self.output)
        self.assertEqual(verified, manifest)
        self.assertEqual(input_path.stat().st_size, tile["input_size"])

    def test_changed_compressed_source_is_refused_and_keeps_lock(self):
        path = self.tile()
        manifest = prepare_module.prepare(self.source, self.output, decoder=fake_decode)
        lock_bytes = (self.output / "sources.json").read_bytes()
        path.write_bytes(b"changed payload")
        with self.assertRaisesRegex(ValueError, "source tile changed"):
            prepare_module.prepare(self.source, self.output, decoder=lambda _data: self.fail("must not decode"))
        self.assertEqual((self.output / "sources.json").read_bytes(), lock_bytes)
        self.assertEqual(len(manifest["tiles"]), 1)

    def test_changed_raw_output_is_refused_by_verify(self):
        self.tile()
        manifest = prepare_module.prepare(self.source, self.output, decoder=fake_decode)
        raw = self.output / manifest["tiles"][0]["path"]
        raw.write_bytes(b"corrupt")
        with self.assertRaisesRegex(ValueError, "raw output missing or changed"):
            prepare_module._verify(self.source, self.output)

    def test_limitzoom_filters_tiles_and_is_pinned(self):
        self.tile(z=1, x=0, y=0)
        self.tile(z=2, x=0, y=0)
        manifest = prepare_module.prepare(self.source, self.output, limitzoom=1, decoder=fake_decode)
        self.assertEqual([tile["z"] for tile in manifest["tiles"]], [1])
        self.assertEqual(prepare_module._verify(self.source, self.output)["limitzoom"], 1)
        with self.assertRaisesRegex(ValueError, "differs from sources.json"):
            prepare_module._verify(self.source, self.output, 2)

    def test_invalid_xyz_is_rejected(self):
        self.tile(z=1, x=2, y=0)
        with self.assertRaisesRegex(ValueError, "outside zoom"):
            prepare_module.discover(self.source, None)

    def test_real_webp_when_pillow_is_available(self):
        try:
            from PIL import Image
        except ImportError:
            self.skipTest("Pillow is not installed; fake decoder tests cover the preparation contract")
        image = Image.new("RGBA", (512, 512), (5, 9, 13, 255))
        import io
        encoded = io.BytesIO()
        image.save(encoded, format="WEBP", lossless=True)
        self.tile(data=encoded.getvalue())
        manifest = prepare_module.prepare(self.source, self.output)
        raw = (self.output / manifest["tiles"][0]["path"]).read_bytes()
        self.assertEqual(raw[:4], bytes((5, 9, 13, 255)))
        self.assertEqual(len(raw), 512 * 512 * 4)


if __name__ == "__main__":
    unittest.main()
