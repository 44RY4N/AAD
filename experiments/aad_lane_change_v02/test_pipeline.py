import csv
import hashlib
import unittest
from pathlib import Path
from aad_analysis.io import read_json
from experiments.aad_lane_change_v01.analyze import schema_check,SCHEMA
from .sweep import configurations
from .evidence import evidence

ROOT=Path(__file__).resolve().parents[2]


class ConcurrentExperimentTests(unittest.TestCase):
    def test_frozen_scenario_matches_stage1_snapshot(self):
        snapshot=read_json(ROOT/'test_output/aad_lc2_sweep_01/controller_snapshot.json')
        hashes={k.replace('\\','/'):v for k,v in snapshot['sha256'].items()}
        path=ROOT/'experiments/aad_lane_change_v02/scenario.py'
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),hashes['experiments/aad_lane_change_v02/scenario.py'])

    def test_factorial_design_and_unique_trial_ids(self):
        configs=configurations();self.assertEqual(len(configs),26)
        self.assertEqual(len({c['scenario_id'] for c in configs}),26)
        self.assertEqual(len({(c['target_vehicle_gap_m'],c['target_vehicle_speed_kmh'],c['command_step']) for c in configs[:18]}),18)

    def test_corrected_desired_speeds_are_real_policy_targets(self):
        for number,speed in [(12,20),(13,40)]:
            path=ROOT/'test_output/aad_lc2_refine_01'/f'lc2_refine_{number:03d}'
            self.assertTrue(all(d['target_speed_kmh']==speed for d in read_json(path/'decisions.json')))

    def test_terminal_sample_retained_without_suppressing_contact(self):
        path=ROOT/'test_output/aad_lc2_refine_01/lc2_refine_007'
        terminal=read_json(path/'termination.json');self.assertTrue(terminal['vehicle_contact'])
        with (path/'trajectory.csv').open(newline='') as file:rows=list(csv.DictReader(file))
        final=[r for r in rows if int(r['step'])==terminal['step']]
        self.assertEqual(len(final),4);self.assertTrue(any(r['crash']=='True' for r in final))
        self.assertFalse(read_json(path/'geometry/geometry_manifest.json')['completed'])

    def test_concurrent_witnesses_are_inside_actual_event(self):
        result=evidence(ROOT/'test_output/aad_lc2_refine_01/lc2_refine_003',ROOT/'analysis_output/aad_lc2_refine_01/lc2_refine_003')
        hits=result['concurrent_model_interaction_samples'];self.assertEqual([h['step'] for h in hits],list(range(60,68)))
        for h in hits:
            self.assertTrue(all(v>0 for v in h['target_lane_occupancy_fractions']))
            self.assertGreater(h['model_ttc_s'],0);self.assertLessEqual(h['model_ttc_s'],5)
            self.assertTrue(result['event_window_steps'][0]<=h['step']<=result['event_window_steps'][1])
        self.assertEqual(result['vehicle_contact_samples'],0)

    def test_representatives_and_annotations_repeat(self):
        root=ROOT/'analysis_output/aad_lc2_representatives_01'
        for name in ['clear','concurrent','delayed']:
            for suffix in ['', '_repeat']:
                annotation=read_json(root/(name+suffix)/'annotated_event.json');schema_check(annotation,read_json(SCHEMA))
                self.assertEqual(annotation['risk_assessment']['level'],'unassessed')
            self.assertEqual(read_json(root/name/'v02/validation.json')['normalized_feature_sha256'],
                read_json(root/(name+'_repeat')/'v02/validation.json')['normalized_feature_sha256'])
        c=read_json(root/'concurrent/window_evidence.json');a=read_json(root/'clear/window_evidence.json');d=read_json(root/'delayed/window_evidence.json')
        self.assertTrue(c['concurrent_model_interaction_samples']);self.assertFalse(a['concurrent_model_interaction_samples'])
        self.assertGreater(d['command_to_departure_steps'],a['command_to_departure_steps'])


if __name__=='__main__':unittest.main()
