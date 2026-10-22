import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import urllib.error
import fetch_glo30 as fetch

class Response(io.BytesIO):
    status=200
    def __init__(self, data, url):
        super().__init__(data);self.url=url;self.headers={'Content-Length':str(len(data)),'ETag':'"test"'}
    def geturl(self):
        return self.url

class Acquisition(unittest.TestCase):
    def test_bbox_budget_and_negative_hemispheres(self):
        self.assertIn('S01_00_W001_00',fetch.entries([-.1,-.1,0,0])[0]['path'])
        self.assertEqual(len(fetch.entries([8.5,49.9,8.6,50.1])),2)
        for bbox in ([0,0,10,10],[180,0,-180,1],[0,float('nan'),1,2]):
            with self.assertRaises(ValueError):fetch.entries(bbox)

    def test_pin_and_offline_reuse(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            with patch.object(fetch.urllib.request,'urlopen',side_effect=lambda r,**kw:Response(b'pinned bytes',r.full_url)) as net:
                first=fetch.run(root,[8.1,49.1,8.2,49.2]);self.assertEqual(net.call_count,1)
            with patch.object(fetch.urllib.request,'urlopen',side_effect=AssertionError('offline')):
                self.assertEqual(first,fetch.run(root,verify=True))
                self.assertEqual(first,fetch.run(root))

    def test_changed_upstream_preserves_old_file_and_lock(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            with patch.object(fetch.urllib.request,'urlopen',side_effect=lambda r,**kw:Response(b'original',r.full_url)):
                manifest=fetch.run(root,[8.1,49.1,8.2,49.2])
            path=root/manifest['files'][0]['path'];path.write_bytes(b'damaged')
            lock=(root/'sources.json').read_bytes()
            with patch.object(fetch.urllib.request,'urlopen',side_effect=lambda r,**kw:Response(b'new upstream',r.full_url)):
                with self.assertRaisesRegex(ValueError,'upstream bytes changed'):fetch.run(root)
            self.assertEqual(path.read_bytes(),b'damaged')
            self.assertEqual((root/'sources.json').read_bytes(),lock)
            self.assertEqual(list(root.glob('*.part')),[])

    def test_404_is_missing_and_503_retries(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            error=urllib.error.HTTPError(fetch.ORIGIN,404,'missing',{},None)
            with patch.object(fetch.urllib.request,'urlopen',side_effect=error):
                m=fetch.run(root,[8.1,49.1,8.2,49.2])
            self.assertEqual(m['files'][0]['status'],'unavailable')
            self.assertFalse((root/m['files'][0]['path']).exists())
        with tempfile.TemporaryDirectory() as d:
            error=urllib.error.HTTPError(fetch.ORIGIN,503,'busy',{},None)
            url=fetch.entries([8.1,49.1,8.2,49.2])[0]['url']
            with patch.object(fetch.urllib.request,'urlopen',side_effect=[error,Response(b'ok',url)]) as net,patch.object(fetch.time,'sleep'):
                fetch.run(Path(d),[8.1,49.1,8.2,49.2]);self.assertEqual(net.call_count,2)

    def test_paths_origins_and_truncation(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for entry in ({'path':'../escape','url':fetch.ORIGIN+'a'},{'path':'ok','url':'https://bad.example/a'}):
                with self.assertRaises(ValueError):fetch.acquire(entry,root)
            url=fetch.ORIGIN+'a';r=Response(b'bad',url);r.headers['Content-Length']='50'
            with patch.object(fetch.urllib.request,'urlopen',return_value=r):
                with self.assertRaisesRegex(ValueError,'truncated'):fetch.acquire({'path':'a.tif','url':url},root)
            self.assertFalse((root/'a.tif').exists())

if __name__=='__main__':unittest.main()
