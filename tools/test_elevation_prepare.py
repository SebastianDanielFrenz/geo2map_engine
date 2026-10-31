"""Native command integration; no network, WebP decoder or external datasets."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

EXE = os.environ.get('G2M_ELEVATION_PREPARE_EXE')

@unittest.skipUnless(EXE, 'set G2M_ELEVATION_PREPARE_EXE to the built native command')
class Preparation(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.rgba=self.root/'rgba';(self.rgba/'0/0').mkdir(parents=True)
        self.manifest=self.rgba/'sources.json';self.write_source(100)

    def tearDown(self):
        self.temp.cleanup()

    def write_source(self,height):
        data=bytes([128,height,0,255])*(512*512)
        (self.rgba/'0/0/0.rgba').write_bytes(data)
        self.manifest.write_text(json.dumps({'format':'g2m.mapterhorn_rgba/1','tile_size':512,'raw_format':'rgba8-row-major',
            'source_metadata':{'surface':'unknown','vertical_reference':'unknown'},
            'tiles':[{'z':0,'x':0,'y':0,'width':512,'height':512,'path':'0/0/0.rgba','size':len(data),'sha256':hashlib.sha256(data).hexdigest()}]}),encoding='utf-8')

    def job(self,scope='first',**extra):
        j={'format':'g2m.elevation_job/1','target':'utm32n/2/455/5403','mapterhorn_manifest':str(self.manifest),
           'output':{'store':str(self.root/'store'),'scope':scope,'layer':'g2m.src.elev.composite'}}
        j.update(extra);return j

    def run_job(self,job):
        path=self.root/'job.json';path.write_text(json.dumps(job),encoding='utf-8')
        return subprocess.run([EXE,str(path)],capture_output=True,text=True,timeout=45)

    def test_repeatability_and_primary_preservation(self):
        job=self.job();a=self.run_job(job);self.assertEqual(a.returncode,0,a.stderr)
        b=self.run_job(job);self.assertEqual(b.returncode,0,b.stderr)
        digest=lambda r:re.search(r'hash=([a-f0-9]{64})',r.stdout)[1]
        self.assertEqual(digest(a),digest(b))
        changed=json.loads(self.manifest.read_text(encoding='utf-8'))
        changed['source_metadata']['surface']='dtm'
        self.manifest.write_text(json.dumps(changed),encoding='utf-8')
        provenance_only=self.run_job(job)
        self.assertNotEqual(provenance_only.returncode,0)
        self.assertIn('provenance differs',provenance_only.stderr)
        self.write_source(100)
        self.write_source(110)
        j=self.job('second',primary={'store':str(self.root/'store'),'scope':'first','layer':'g2m.src.elev.composite'})
        c=self.run_job(j);self.assertEqual(c.returncode,0,c.stderr)
        self.assertIn('primary_preserved=1048576',c.stdout)
        self.assertEqual(digest(a),digest(c))
        changed=self.run_job(job);self.assertNotEqual(changed.returncode,0)
        self.assertIn('choose a new scope',changed.stderr)
        self.write_source(100);self.assertEqual(self.run_job(job).returncode,0)

    def test_existing_writer_claim_is_respected(self):
        store=self.root/'store';store.mkdir();claim=store/'.elevation_prepare_lock';claim.mkdir()
        r=self.run_job(self.job());self.assertNotEqual(r.returncode,0)
        self.assertIn('another preparation writer',r.stderr)
        self.assertTrue(claim.is_dir());self.assertFalse((store/'tiles.sqlite3').exists())

    def test_tampered_pixels_fail_before_store_creation(self):
        (self.rgba/'0/0/0.rgba').write_bytes(bytes([0])*1048576)
        r=self.run_job(self.job());self.assertNotEqual(r.returncode,0)
        self.assertIn('hash mismatch',r.stderr);self.assertFalse((self.root/'store').exists())

    def test_bad_jobs_and_primary_overwrite_are_rejected(self):
        bad=self.job(blend_radius_m=257);r=self.run_job(bad)
        self.assertNotEqual(r.returncode,0);self.assertIn('radius',r.stderr)
        j=self.job();self.assertEqual(self.run_job(j).returncode,0)
        j['primary']={'store':str(self.root/'store'),'scope':'first','layer':'g2m.src.elev.composite'}
        r=self.run_job(j);self.assertNotEqual(r.returncode,0);self.assertIn('never overwrite primary',r.stderr)

if __name__=='__main__':unittest.main()
