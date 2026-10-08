"""Offline ABI/CRC/time/flags/roundtrip tests; no robot or network access."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import zlib
from unittest.mock import patch, MagicMock

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location("collector",ROOT/"pc/collector.py")
collector=importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)

def capture(times=(5000,55000,105000), bits=(7,5,7)):
    records=[]
    for i,(t,flag) in enumerate(zip(times,bits)):
        sequence=1 if i<2 else 2
        records.append(collector.RECORD.pack(t,10,sequence,40.,40.1,0.,0.,40.,0.,1.1,-.24,12.0,384,80,2,flag))
    payload=b''.join(records)
    return collector.HEADER.pack(b'SQDCAP1\0',1,56,len(records),50000,zlib.crc32(payload),0,0,
             .09,.01,.6,.75,4.5,1.1,.03,40.)+payload

class Tests(unittest.TestCase):
    def test_fresh_and_repeat(self):
        rows,meta=collector.parse_capture(capture())
        self.assertEqual([r['depth_fresh'] for r in rows],[1,0,1])
        self.assertEqual([r['depth_sequence'] for r in rows],[1,1,2])
        self.assertFalse(meta['actuator_feedback'])
    def test_corruption(self):
        bad=bytearray(capture());bad[-1]^=1
        with self.assertRaisesRegex(ValueError,'CRC32'):collector.parse_capture(bytes(bad))
    def test_truncated(self):
        with self.assertRaises(ValueError):collector.parse_capture(capture()[:-1])
    def test_bad_magic(self):
        with self.assertRaises(ValueError):collector.parse_capture(b'WRONGABI'+capture()[8:])
    def test_time_order(self):
        with self.assertRaises(ValueError):collector.parse_capture(capture(times=(55000,5000,105000)))
    def test_fresh_requires_valid(self):
        with self.assertRaises(ValueError):collector.parse_capture(capture(bits=(2,5,7)))
    def test_roundtrip_and_no_overwrite(self):
        # Temporary outputs are confined to this new project's tests folder.
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as directory:
            prefix=Path(directory)/'fixture'
            paths=collector.save_capture(capture(),prefix)
            self.assertEqual(paths[0].read_bytes(),capture())
            self.assertIn('pwm_commanded',paths[1].read_text(encoding='utf-8'))
            with self.assertRaises(FileExistsError):collector.save_capture(capture(),prefix)
    def test_invalid_input_creates_no_files(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as directory:
            prefix=Path(directory)/'bad'
            with self.assertRaises(ValueError):collector.save_capture(b'bad',prefix)
            self.assertEqual(list(Path(directory).iterdir()),[])
    def test_http_methods_without_network(self):
        response=MagicMock()
        response.__enter__.return_value.read.return_value=b'{"state":"idle"}'
        opener=MagicMock()
        opener.open.return_value=response
        with patch.object(collector.urllib.request,'build_opener',return_value=opener):
            collector.request('http://127.0.0.1','status')
            req=opener.open.call_args.args[0]
            self.assertEqual(req.get_method(),'GET')
            self.assertEqual(req.full_url,'http://127.0.0.1/capture/status')
            collector.request('http://127.0.0.1','stop',{})
            req=opener.open.call_args.args[0]
            self.assertEqual(req.get_method(),'POST')
            self.assertEqual(req.data,b'{}')
            self.assertEqual(req.full_url,'http://127.0.0.1/capture/stop')
    def test_reject_non_http_address(self):
        with self.assertRaises(ValueError):collector.request('https://127.0.0.1','status')

if __name__=='__main__':unittest.main()
