"""Reproduce loadcell starvation using the production SEDSnet bounded queue."""
import json
import pathlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class LoadcellQueuePriorityTests(unittest.TestCase):
    def test_fast_loadcells_have_equal_priority_below_commands(self):
        types = {x['name']: x for x in json.loads((ROOT / 'config/sedsnet.json').read_text())['types']}
        priorities = {types[n]['priority'] for n in ('KG1000', 'KG50', 'DAQ_KG50_SELECTED')}
        self.assertEqual(len(priorities), 1, 'P6 must not permanently displace the other loadcell')
        self.assertLess(priorities.pop(), types['UMBILICAL_STATUS']['priority'])

    def test_p6_saturation_cannot_refuse_kg1000_but_commands_stay_protected(self):
        # Use SDK source from the workspace or the build's fetched dependency.
        paths = [ROOT.parent / 'SEDSnet/src/queue.rs']
        paths += list((ROOT / 'build').glob('*/_deps/sedsnet-*/**/src/queue.rs'))
        sdk = next((p for p in paths if p.is_file()), None)
        if not sdk or not shutil.which('rustc'):
            self.skipTest('requires fetched SEDSnet source and host Rust compiler')
        types = {x['name']: x for x in json.loads((ROOT / 'config/sedsnet.json').read_text())['types']}
        source = r'''
extern crate alloc;
#[derive(Debug,PartialEq)] pub enum TelemetryError { Io(&'static str), PacketTooLarge(&'static str), BadArg }
pub type TelemetryResult<T> = Result<T,TelemetryError>;
mod memory_admission { pub fn check(_:usize,_:usize)->crate::TelemetryResult<()> { Ok(()) } }
#[path="QUEUE"] mod queue;
#[derive(Clone,Debug)] struct Item { id:u32, priority:u8 }
impl queue::ByteCost for Item { fn byte_cost(&self)->usize { 8 } }
fn main() {
 let mut q=queue::BoundedDeque::new(64,64,1.0);
 for _ in 0..8 { q.push_back_prioritized(Item{id:50,priority:P50},|p|p.priority).unwrap(); }
 q.push_back_prioritized(Item{id:1000,priority:P1000},|p|p.priority)
   .expect("1000 kg cannot be permanently rejected by P6 samples");
 assert!(q.iter().any(|p|p.id==1000));
 q.push_back_prioritized(Item{id:200,priority:200},|p|p.priority).unwrap();
 for _ in 0..64 { q.push_back_prioritized(Item{id:50,priority:P50},|p|p.priority).unwrap(); }
 assert!(q.iter().any(|p|p.id==200),"telemetry must not evict valve status or commands");
}
'''.replace('QUEUE', str(sdk)).replace('P50', str(types['DAQ_KG50_SELECTED']['priority'])).replace('P1000', str(types['KG1000']['priority']))
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / 'test.rs'
            path.write_text(source)
            binary = pathlib.Path(tmp) / 'test'
            subprocess.run(['rustc', '--edition=2024', '-A', 'dead_code', str(path), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
