import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('autonomy_trace', Path(__file__).parent/'autonomy_sim/trace.py')
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)


class TraceTests(unittest.TestCase):
    def read(self, records, fps=2, tail=''):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'trace.jsonl'
            path.write_text(''.join(json.dumps(r)+'\n' for r in records)+tail)
            return trace.read_recording(path, fps)

    def test_portable_coordinates_and_final_sample(self):
        pose = {'position': [1, 2, 3], 'orientation': [0, 0, .6, .8]}
        header = {'t':'header', 'scenario': {'bodies': [
            {'id': 0, 'name': 'world', 'motion': 0},
            {'id': 1, 'name': 'hydrus', 'motion': 1},
            {'id': 2, 'name': 'gate', 'motion': 2, 'tracked': True}], 'camera': {'body': 1}}}
        samples = [{'t':'sample', 'step': i, 'timestamp_ns': i*100000000,
                    'bodies': [{'id': 1, 'pose': pose}, {'id': 2, 'pose': pose}],
                    'camera': pose, 'thr': [.1]*8} for i in range(7)]
        result = self.read([header, *samples, {'t':'end', 'step':6, 'status':1}])
        self.assertEqual([r['step'] for r in result['samples']], [0, 5, 6])
        self.assertEqual(result['header']['objects'], ['hydrus','gate','camera'])
        self.assertEqual(result['samples'][0]['poses']['hydrus'], [-1,2,-3,.8,0,0,-.6])
        self.assertEqual(result['end']['reason'], 'tick budget')
        self.assertAlmostEqual(result['end']['time'], .6)

    def test_experimental_recording_and_partial_tail(self):
        sample = {'t':'sample','step':0,'time':0,'poses':{'hydrus':[1,2,3,1,0,0,0]},'thr':[0]*8}
        result = self.read([{'t':'header','objects':['hydrus']}, sample], tail='{"t":')
        self.assertEqual(result['samples'], [sample])
        self.assertIsNone(result['end'])

    def test_reject_bad_records(self):
        with self.assertRaises(ValueError):
            self.read([{'t':'sample'}])
        with self.assertRaises(ValueError):
            self.read([{'t':'header'}], tail='invalid\n')


if __name__ == '__main__':
    unittest.main()
