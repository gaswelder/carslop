import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zlib

@unittest.skipUnless(importlib.util.find_spec('PIL'),'optional Pillow required for aerial compiler')
class AerialTilesTests(unittest.TestCase):
    def test_neighbour_gutters_share_pixels_and_payloads_match_checksums(self):
        from PIL import Image
        from build_aerial_tiles import build
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);image=Image.new('RGB',(128,128))
            image.putdata([(x,y,(x+y)%256) for y in range(128) for x in range(128)])
            image.save(root/'source.png')
            (root/'export.json').write_text(json.dumps({'extent':dict(xmin=3709000,ymin=4185800,xmax=3709100,ymax=4185900)}))
            build(root/'source.png',root/'export.json',root/'tiles',size=64,gutter=4)
            lines=(root/'tiles/tiles.txt').read_text().splitlines()[2:]
            self.assertEqual(len(lines),4)
            for line in lines:
                f=line.split();raw=(root/'tiles'/f[5]).read_bytes()
                self.assertEqual(len(raw),72*72*3)
                self.assertEqual(zlib.crc32(raw),int(f[6],16))
            a=Image.frombytes('RGB',(72,72),(root/'tiles/0_0.rgb').read_bytes())
            b=Image.frombytes('RGB',(72,72),(root/'tiles/1_0.rgb').read_bytes())
            self.assertEqual(a.crop((64,0,72,72)).tobytes(),b.crop((0,0,8,72)).tobytes())
            self.assertEqual(a.getpixel((0,0)),image.getpixel((0,0)))
            self.assertEqual(a.getpixel((4,4)),image.getpixel((0,0)))
