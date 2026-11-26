import copy
import unittest
from pathlib import Path
from aad_analysis.io import read_json
from .analyze import ROOT,SCHEMA,schema_check


class ScenarioPipelineTests(unittest.TestCase):
    def test_matched_controls(self):
        a=read_json(Path(__file__).with_name('ordinary.json'));b=read_json(Path(__file__).with_name('interaction.json'))
        changed={k for k in a if a[k]!=b[k]}
        self.assertEqual(changed,{'scenario_id','condition_intent','target_vehicle_gap_m','target_vehicle_speed_kmh'})

    def test_actual_annotations_and_references(self):
        for name in ['ordinary','interaction','ordinary_repeat','interaction_repeat']:
            p=ROOT/'analysis_output'/('aad_lc_'+name+'_01');a=read_json(p/'event_annotation.json')
            schema_check(a,read_json(SCHEMA))
            self.assertEqual(a['risk_assessment']['level'],'unassessed')
            self.assertGreaterEqual(a['step_range']['end'],a['step_range']['start'])
            self.assertTrue(a['verification']['physical_event_verified'])
            self.assertFalse(a['verification']['criticality_verified'])
            ids={r['actor_id'] for r in a['actors']}
            self.assertTrue(all(set(m['actor_ids'])<=ids for m in a['metrics']))

    def test_reproducible_trajectories_and_features(self):
        for name in ['ordinary','interaction']:
            a=ROOT/'analysis_output'/('aad_lc_'+name+'_01');b=ROOT/'analysis_output'/('aad_lc_'+name+'_repeat_01')
            self.assertEqual(read_json(a/'summary.json')['observed_event'],read_json(b/'summary.json')['observed_event'])
            for version,field in [('v01','feature_fingerprint_without_generated_ids'),('v02','normalized_feature_sha256')]:
                self.assertEqual(read_json(a/version/'validation.json')[field],read_json(b/version/'validation.json')[field])

    def test_schema_rejects_misleading_labels_and_invalid_nulls(self):
        s=read_json(SCHEMA);a=copy.deepcopy(s['examples'][0]);a['risk_assessment']['level']='dangerous'
        with self.assertRaises(AssertionError):schema_check(a,s)
        a=copy.deepcopy(s['examples'][0]);a['metrics'][0]['value']=0
        with self.assertRaises(AssertionError):schema_check(a,s)


if __name__=='__main__':unittest.main()
