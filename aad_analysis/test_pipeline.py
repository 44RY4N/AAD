"""Analytical and input-integrity tests; no simulator launch."""
import copy
import csv
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from .features import actor_features, derivative, pair_features
from .io import load_run
from .run import extract


def track():
    # s=t^3, l=2t^2: exact central derivative checks with signed lateral motion.
    return {i:dict(step=i, time_s=i*.1, x=(i*.1)**3, y=-2*(i*.1)**2,
                   source_longitudinal_m=(i*.1)**3, source_lateral_m=2*(i*.1)**2,
                   speed_ms=3*(i*.1)**2, lane=(">>",">>>",1), on_lane=True, same_road=True,
                   heading_rad=0.) for i in range(9)}


class FeatureTests(unittest.TestCase):
    def test_analytic_polynomials_units_and_sign(self):
        t=track(); f=actor_features(t,4,.1)
        self.assertAlmostEqual(f["j_longitudinal"]["value"],6.)
        self.assertAlmostEqual(f["a_longitudinal"]["value"],2.4)
        self.assertAlmostEqual(f["v_lateral"]["value"],1.6)
        self.assertAlmostEqual(f["a_lateral"]["value"],4.)
        self.assertAlmostEqual(f["j_lateral"]["value"],0.)
        self.assertAlmostEqual(f["j_speed"]["value"],6.)
        self.assertEqual(f["j_speed"]["unit"],"m/s^3")
        self.assertAlmostEqual(f["vy_world"]["value"],-1.6)

    def test_boundary_gap_localization_and_road_change(self):
        t=track()
        self.assertEqual(derivative(t,0,"x",1,.1)["reason"],"boundary_support")
        del t[3]
        self.assertEqual(derivative(t,4,"x",1,.1)["reason"],"missing_sample")
        t=track();t[3]["on_lane"]=False
        self.assertEqual(derivative(t,4,"x",1,.1)["validity"],"invalid")
        t=track();t[3]["lane"]=("next","road",1)
        self.assertEqual(derivative(t,4,"x",1,.1)["reason"],"road_transition")
        t=track();t[3]["time_s"]+=.01
        self.assertEqual(derivative(t,4,"x",1,.1)["reason"],"nonuniform_time")

    def test_point_proxies_are_not_footprint_metrics(self):
        a=track(); b=copy.deepcopy(a)
        for r in a.values():r["x"]=r["source_longitudinal_m"]=10*r["time_s"]
        for r in b.values():r["x"]=r["source_longitudinal_m"]=20+5*r["time_s"]
        af,bf=actor_features(a,4,.1),actor_features(b,4,.1)
        p=pair_features(a[4],b[4],af,bf)
        self.assertAlmostEqual(p["reference_distance"]["value"],18.)
        self.assertAlmostEqual(p["point_following_ttc_proxy"]["value"],3.6)
        self.assertAlmostEqual(p["gross_reference_headway"]["value"],1.8)
        self.assertIsNone(p["footprint_2d_ttc"]["value"])
        self.assertEqual(p["minimum_footprint_clearance"]["validity"],"unavailable")
        p=pair_features(a[4],b[4],af,af)
        self.assertEqual(p["point_following_ttc_proxy"]["reason"],"no_positive_closing_speed")
        still=copy.deepcopy(af);still["v_longitudinal"]["value"]=0
        p=pair_features(a[4],b[4],still,bf)
        self.assertEqual(p["gross_reference_headway"]["validity"],"undefined")
        b[4]["lane"]=(">>",">>>",0)
        self.assertEqual(pair_features(a[4],b[4],af,bf)["point_following_ttc_proxy"]["validity"],"not_applicable")


class InputTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)/"input"
        (self.root/"camera/frames").mkdir(parents=True)
        source=Path(__file__).resolve().parents[1]/"test_output/rgb_extended_02"
        m=json.loads((source/"metadata.json").read_text());m["settings"]["steps"]=4
        (self.root/"metadata.json").write_text(json.dumps(m))
        with (source/"trajectory.csv").open() as f:
            self.rows=list(csv.DictReader(f))[:20]
        self.write_rows()
        self.frames=[json.loads(line) for line in (source/"camera/frames.jsonl").read_text().splitlines()][:5]
        self.write_frames()
        for frame in self.frames:
            shutil.copyfile(source/"camera"/frame["file"],self.root/"camera"/frame["file"])

    def tearDown(self):
        self.temp.cleanup()

    def write_rows(self):
        with (self.root/"trajectory.csv").open("w",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=self.rows[0]);writer.writeheader();writer.writerows(self.rows)

    def write_frames(self):
        (self.root/"camera/frames.jsonl").write_text("\n".join(json.dumps(r) for r in self.frames))

    def test_input_contract_and_duplicate_rejection(self):
        self.assertEqual(len(load_run(self.root)[2]),20)
        self.rows.append(self.rows[0]);self.write_rows()
        with self.assertRaisesRegex(ValueError,"Duplicate"):load_run(self.root)

    def test_misaligned_metadata_rejected(self):
        self.frames[2]["subject"]["x"]+=1;self.write_frames()
        with self.assertRaisesRegex(ValueError,"mismatch"):load_run(self.root)

    def test_invalid_boolean_and_nonfinite_rejected(self):
        self.rows[0]["on_lane"]="false";self.write_rows()
        with self.assertRaisesRegex(ValueError,"boolean"):load_run(self.root)
        self.rows[0]["on_lane"]="True";self.rows[0]["x"]="nan";self.write_rows()
        with self.assertRaisesRegex(ValueError,"Non-finite"):load_run(self.root)

    def test_existing_output_and_nested_output_rejected(self):
        with self.assertRaises(ValueError):extract(self.root,self.root/"analysis")
        existing=Path(self.temp.name)/"existing_analysis";existing.mkdir()
        with self.assertRaises(FileExistsError):extract(self.root,existing)


class IntegrationTests(unittest.TestCase):
    def test_existing_rgb_repeats_and_manifest_boundaries(self):
        repo=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            reports=[]
            for name in ['rgb_extended_02','rgb_extended_03']:
                output=Path(temp)/name
                report=extract(repo/'test_output'/name,output)
                reports.append(report)
                self.assertEqual(report['actor_feature_records'],1204)
                self.assertEqual(report['subject_pair_records'],903)
                self.assertTrue(report['source_unchanged'])
                manifest=json.loads((output/'manifest.json').read_text())
                self.assertFalse(manifest['geometry']['resolved_actor_footprints_available'])
                self.assertTrue(all(a['recorded_dimensions_m'] is None for a in manifest['actors']))
                with (output/'pair_features.jsonl').open() as file:
                    for line in file:
                        pair=json.loads(line)
                        self.assertEqual(pair['risk_assessment'],'unassessed')
                        self.assertIsNone(pair['metrics']['footprint_2d_ttc']['value'])
            self.assertEqual(reports[0]['feature_fingerprint_without_generated_ids'],reports[1]['feature_fingerprint_without_generated_ids'])


if __name__ == "__main__":
    unittest.main()
