"""Hand-checkable geometry contracts, separate from simulator integration tests."""
import math
import json
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest

from panda3d.bullet import BulletBoxShape,BulletRigidBodyNode,BulletSphereShape
from panda3d.core import NodePath,PerspectiveLens,TransformState,Vec3

from experiments.geometry_math import box_vertices,convex_hull_xy,polygon_clearance_for_tests,transform_point
from experiments.geometry_recording import camera_calibration,matrix,probe_box_faces,shapes_for
from experiments.validate_geometry import validated_data


def pose(heading,x=0,y=0,z=0):
    c,s=math.cos(heading),math.sin(heading)
    return [[c,s,0,0],[-s,c,0,0],[0,0,1,0],[x,y,z,1]]


class GeometryTests(unittest.TestCase):
    def test_heading_zero_vehicle_axes(self):
        # Vehicle heading zero corresponds to Panda chassis heading -90 degrees.
        self.assertAlmostEqual(transform_point([0,2,0],pose(-math.pi/2))[0],2)
        self.assertAlmostEqual(transform_point([0,2,0],pose(-math.pi/2))[1],0)

    def test_rotated_heading(self):
        self.assertEqual(transform_point([0,2,0],pose(0)),[0,2,0])
        p=transform_point([1,0,0],pose(math.pi/2))
        self.assertAlmostEqual(p[0],0);self.assertAlmostEqual(p[1],1)

    def test_translated_vehicle(self):
        self.assertEqual(transform_point([1,2,3],pose(0,10,20,30)),[11,22,33])

    def test_nonzero_shape_reference_offset(self):
        local=transform_point([1,2,.5],pose(0,.2,-.3,.5))
        self.assertEqual(local,[1.2,1.7,1.])
        world=transform_point(local,pose(-math.pi/2,10,20,0))
        self.assertAlmostEqual(world[0],11.7);self.assertAlmostEqual(world[1],18.8)

    def test_separated_rectangles(self):
        a=[[-1,-1],[1,-1],[1,1],[-1,1]]
        b=[[4,-1],[6,-1],[6,1],[4,1]]
        self.assertEqual(polygon_clearance_for_tests(a,b),3.)

    def test_touching_rectangles(self):
        a=[[-1,-1],[1,-1],[1,1],[-1,1]]
        b=[[1,-1],[3,-1],[3,1],[1,1]]
        self.assertEqual(polygon_clearance_for_tests(a,b),0.)

    def test_overlapping_rectangles(self):
        a=[[-1,-1],[1,-1],[1,1],[-1,1]]
        b=[[0,-1],[2,-1],[2,1],[0,1]]
        self.assertEqual(polygon_clearance_for_tests(a,b),0.)

    def test_actual_box_extents_offset_and_bullet_face_probes(self):
        body=BulletRigidBodyNode("known_box")
        body.addShape(BulletBoxShape(Vec3(1,2,.5)),TransformState.makePos(Vec3(.2,-.3,.5)))
        fake=SimpleNamespace(chassis=NodePath(body))
        shape=shapes_for(fake)[0]
        self.assertEqual(shape["half_extents_with_margin_m"],[1.,2.,.5])
        self.assertAlmostEqual(shape["centre_offset_actor_m"][0],.2)
        xs=[p[0] for p in shape["actor_local_footprint_xy_m"]]
        self.assertAlmostEqual(min(xs),-.8);self.assertAlmostEqual(max(xs),1.2)
        self.assertLess(probe_box_faces(fake)[0]["max_error_m"],1e-4)

    def test_unsupported_shape_remains_unavailable(self):
        body=BulletRigidBodyNode("sphere");body.addShape(BulletSphereShape(1))
        shape=shapes_for(SimpleNamespace(chassis=NodePath(body)))[0]
        self.assertEqual(shape["status"],"unavailable")
        self.assertIsNone(shape["actor_local_vertices_m"])

    def test_projection_uses_all_3d_vertices(self):
        # Pitch/roll can make upper vertices extend the XY projection.
        c,s=math.cos(math.pi/4),math.sin(math.pi/4)
        rotation=[[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]]
        points=[transform_point(p,rotation) for p in box_vertices([1,2,1])]
        hull=convex_hull_xy(points)
        self.assertAlmostEqual(max(p[0] for p in hull),math.sqrt(2))

    def test_pinhole_intrinsics_and_native_matrix(self):
        lens=PerspectiveLens();lens.setFov(65);lens.setAspectRatio(640/360)
        cal=camera_calibration(SimpleNamespace(lens=lens,BUFFER_W=640,BUFFER_H=360))
        self.assertAlmostEqual(cal["intrinsics"]["fx_px"],320/math.tan(math.radians(32.5)),places=3)
        self.assertEqual(cal["intrinsics"]["cx_px"],320)
        self.assertEqual(cal["intrinsics"]["cy_px"],180)
        self.assertIsNone(camera_calibration(None)["intrinsics"])


SAVED=Path(__file__).resolve().parents[1]/"test_output/geometry_only_validation_01"


@unittest.skipUnless((SAVED/"geometry/geometry_manifest.json").exists(),"Run the geometry-only integration example first")
class SavedGeometryValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.output=Path(self.temp.name)
        shutil.copytree(SAVED/"geometry",self.output/"geometry")
        shutil.copyfile(SAVED/"trajectory.csv",self.output/"trajectory.csv")
        self.path=self.output/"geometry/geometry_frames.jsonl"
        self.frames=[json.loads(line) for line in self.path.read_text().splitlines()]

    def tearDown(self):self.temp.cleanup()

    def save(self):self.path.write_text("\n".join(json.dumps(f) for f in self.frames))

    def test_saved_runtime_contract(self):
        report,_,_=validated_data(self.output)
        self.assertEqual(report["samples"],301)
        self.assertEqual(report["unsupported_static_shapes"],0)

    def test_missing_geometry_step_rejected(self):
        del self.frames[12];self.save()
        with self.assertRaisesRegex(ValueError,"steps"):validated_data(self.output)

    def test_bad_transformed_footprint_rejected(self):
        self.frames[8]["actors"][0]["footprint_components"][0]["world_vertices_m"][0][0]+=1
        self.save()
        with self.assertRaisesRegex(ValueError,"transform mismatch"):validated_data(self.output)

    def test_wrong_identity_rejected(self):
        self.frames[10]["actors"][0]["actor_id"]="different-id";self.save()
        with self.assertRaisesRegex(ValueError,"actor"):validated_data(self.output)

    def test_wrong_velocity_units_rejected(self):
        self.frames[150]["actors"][0]["velocity_world_m_s"][0]*=3.6;self.save()
        with self.assertRaisesRegex(ValueError,"Velocity units"):validated_data(self.output)

    def test_wrong_timestamp_rejected(self):
        self.frames[10]["time_s"]+=.1;self.save()
        with self.assertRaisesRegex(ValueError,"timestamp"):validated_data(self.output)


if __name__=="__main__":unittest.main()
