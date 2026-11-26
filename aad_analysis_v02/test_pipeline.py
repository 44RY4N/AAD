import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from aad_analysis.io import sha256
from .inputs import load
from .measurements import actor_record,context,pair_record,prepare_lanes
from .polygons import polygon
from .run import extract
from .bullet_study import study


ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/"test_output/geometry_only_validation_01"


class InputAndContextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.original=load(SOURCE)

    def test_geometry_required_for_historical_inputs(self):
        with self.assertRaisesRegex(ValueError,"sidecar required"):load(ROOT/"test_output/rgb_extended_02")

    def test_nonexistent_or_nested_output_protection(self):
        with self.assertRaises(ValueError):extract(SOURCE,SOURCE/"bad")
        with self.assertRaises(FileExistsError):extract(SOURCE,ROOT/"analysis_output")

    def test_invalid_prediction_horizon(self):
        with self.assertRaises(ValueError):extract(SOURCE,ROOT/"unused",float('inf'))

    def test_component_missingness_not_silently_ignored(self):
        run=copy.deepcopy(self.original);f=run["data"]["frames"][10];actor=f["actors"][0]
        run["component_reasons"][actor["actor_id"],0]="unsupported_collision_shape"
        record,occ,g=actor_record(run,actor,f,prepare_lanes(run["data"]["road"]))
        self.assertEqual(record["geometry_coverage"],"unavailable")
        self.assertIsNone(record["metrics"][0]["value"])
        self.assertTrue(all(l['metrics'][0]['value'] is None for l in occ['lanes']))

    def test_invalid_localization_prevents_geometry_interpretation(self):
        run=copy.deepcopy(self.original);f=run["data"]["frames"][10];actor=f["actors"][0]
        run["rows"][10,actor["actor_id"]]["on_lane"]=False
        a,o,g=actor_record(run,actor,f,prepare_lanes(run["data"]["road"]))
        self.assertEqual(a["metrics"][0]["validity"],"invalid");self.assertIsNone(a["metrics"][0]["value"])

    def test_vertical_and_road_context_guards(self):
        run=copy.deepcopy(self.original);f=run["data"]["frames"][10];a,b=f["actors"][:2];lanes=prepare_lanes(run["data"]["road"])
        _,_,ga=actor_record(run,a,f,lanes);_,_,gb=actor_record(run,b,f,lanes)
        for c in b["footprint_components"]:
            for p in c["world_vertices_m"]:p[2]+=10
        self.assertEqual(context(a,b,ga,gb)[1],"vertically_disjoint_outer_envelopes")
        gb["row"]["lane"]=("other","road",0)
        self.assertIn("different_directed_roads",context(a,b,ga,gb)[1])

    def test_curved_and_nonsimple_lanes_unavailable(self):
        road=copy.deepcopy(self.original["data"]["road"])
        lane=road["lanes"][0];lane["python_class"]="example.CircularLane"
        prepared=prepare_lanes(road);self.assertEqual(prepared[tuple(lane["lane_id"])]["status"],"unavailable")
        lane["python_class"]="example.StraightLane";lane["polygon"]["xy_m"]=[[0,0],[1,1],[0,1],[1,0]]
        self.assertEqual(prepare_lanes(road)[tuple(lane["lane_id"])]["status"],"unavailable")

    def test_directional_gap_uses_vertices_not_nominal_length(self):
        run=copy.deepcopy(self.original);f=run["data"]["frames"][0];a,b=f["actors"][:2];lanes=prepare_lanes(run["data"]["road"])
        _,_,ga=actor_record(run,a,f,lanes);_,_,gb=actor_record(run,b,f,lanes)
        pair=pair_record(run,a,b,f,ga,gb,lanes,5)
        value=next(m for m in pair["metrics"] if m["name"]=="shared_path_signed_longitudinal_envelope_gap")
        self.assertEqual(value["validity"],"valid")
        # First source actors are subject x=50 and leader x=74, with yaw-converted box extents.
        expected=min(p[0] for poly in gb['components'].values() for p in poly)-max(p[0] for poly in ga['components'].values() for p in poly)
        self.assertAlmostEqual(value['value'],expected)
        self.assertNotEqual(value['value'],24.)


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.path=Path(self.temp.name)/"source";self.path.mkdir()
        shutil.copytree(SOURCE/"geometry",self.path/"geometry")
        for name in ["trajectory.csv","metadata.json"]:shutil.copyfile(SOURCE/name,self.path/name)

    def tearDown(self):self.temp.cleanup()

    def test_failed_saved_validation_rejected(self):
        p=self.path/"geometry/validation.json";d=json.loads(p.read_text());d["passed"]=False;p.write_text(json.dumps(d))
        with self.assertRaisesRegex(ValueError,"Successful saved"):load(self.path)

    def test_tampered_artifact_rejected(self):
        p=self.path/"geometry/road_geometry.json";p.write_text(p.read_text()+" ")
        with self.assertRaisesRegex(ValueError,"hash mismatch"):load(self.path)

    def test_inconsistent_units_rejected_even_with_updated_hash(self):
        p=self.path/"geometry/geometry_manifest.json";d=json.loads(p.read_text());d["units"]["velocity"]="km/h";p.write_text(json.dumps(d))
        report=self.path/"geometry/validation.json";r=json.loads(report.read_text());r["artifact_hashes"][p.name]=sha256(p);report.write_text(json.dumps(r))
        with self.assertRaisesRegex(ValueError,"units"):load(self.path)


class BulletTests(unittest.TestCase):
    def test_independent_box_contacts_margin_and_static_filter(self):
        result=study();self.assertTrue(result["passed"]);self.assertEqual(len(result["cases"]),30)
        self.assertFalse(result["independent_positive_distance_available"])
        self.assertEqual(result["static_filter_control"]["bullet_contact_records"],0)
        z=next(r for r in result["cases"] if r["case"]["name"]=="elevation_disjoint")
        self.assertEqual(z["outer_envelope"]["relation"],"overlapping");self.assertEqual(z["bullet_contact_records"],0)


if __name__=="__main__":unittest.main()
