import gzip
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from merge_osm import merge, read

class MergeTests(unittest.TestCase):
    def test_preserves_local_objects_but_keeps_newer_versioned_extension_objects(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);original=root/'old.osm.gz';extension=root/'extension.osm';output=root/'merged.osm'
            original.write_bytes(gzip.compress(b'<osm><node id="1" version="2" lat="35.16" lon="33.32"><tag k="natural" v="tree"/></node><node id="2" version="1" lat="35.16" lon="33.32"/></osm>'))
            extension.write_text('<osm><node id="1" lat="35.17" lon="33.33"/><node id="2" version="3" lat="35.18" lon="33.34"/><node id="3" lat="35.19" lon="33.35"/><way id="10"><nd ref="1"/><nd ref="3"/></way></osm>')
            merge(original,extension,output)
            nodes={e.attrib['id']:e for e in ET.parse(output).getroot().findall('node')}
            self.assertEqual(len(nodes),3)
            self.assertEqual(nodes['1'].attrib['lat'],'35.16')
            self.assertEqual(nodes['1'].find('tag').attrib['v'],'tree')
            self.assertEqual(nodes['2'].attrib['version'],'3')
            self.assertEqual([n.attrib['ref'] for n in ET.parse(output).getroot().find('way').findall('nd')],['1','3'])
            before=output.read_bytes();merge(original,extension,output);self.assertEqual(before,output.read_bytes())

    def test_xml_timeout_cannot_be_merged_or_imported_as_a_complete_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);original=root/'old.osm';partial=root/'partial.osm';output=root/'output.osm'
            original.write_text('<osm/>')
            partial.write_text('<osm><remark>runtime error: timeout</remark></osm>')
            with self.assertRaisesRegex(ValueError,'incomplete'):read(partial)
            with self.assertRaisesRegex(ValueError,'incomplete'):merge(original,partial,output)
            self.assertFalse(output.exists())

if __name__=='__main__':unittest.main()
