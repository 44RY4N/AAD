import copy
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from panda3d.core import NodePath,PerspectiveLens,Point3
from experiments.geometry_recording import camera_calibration,matrix
from experiments.geometry_math import transform_point
from .camera import validate_cameras,ground_point
from .controllers import abort_due
from .event_contract import schema,validate
from .events import braking_window,confirm
from .proxy import vehicle_config
from .vision import observations
from .review import validate_responses


class ProtocolTests(unittest.TestCase):
    def camera(self):
        return json.loads((Path(__file__).parent/'configs/roadside_wide.json').read_text())
    def test_fractional_sampling_rejected(self):
        c=self.camera();c[0]['fps']=3
        with self.assertRaises(ValueError):validate_cameras(c,.1)
    def test_fixed_config_and_lower_fps(self):
        c=self.camera();validate_cameras(c,.1);c[0]['fps']=5;validate_cameras(c,.1)
    def test_follow_actor_configuration_rejected(self):
        c=self.camera();c[0]['follow_actor']='subject'
        with self.assertRaises(ValueError):validate_cameras(c,.1)
    def test_duplicate_camera_identity_rejected(self):
        c=self.camera()
        with self.assertRaises(ValueError):validate_cameras(c+c,.1)
    def test_camera_nan_rejected(self):
        c=self.camera();c[0]['position_world_m'][0]=float('nan')
        with self.assertRaises(ValueError):validate_cameras(c,.1)
    def test_ground_projection_roundtrip(self):
        world=NodePath('world');cam=world.attachNewNode('camera');cam.setPos(10,-15,20);cam.lookAt(Point3(5,2,0))
        lens=PerspectiveLens();lens.setAspectRatio(16/9);lens.setFov(90)
        c=camera_calibration(SimpleNamespace(lens=lens,BUFFER_W=1280,BUFFER_H=720))
        c.update(camera_to_world_matrix=matrix(cam.getMat(world)),world_to_camera_matrix=matrix(world.getMat(cam)),ground_plane_world_z_m=0.)
        for p in [[0,0,0],[20,15,0],[-10,-2,0]]:
            ndc=transform_point(transform_point(p,c['world_to_camera_matrix']),c['projection_matrix'])
            recovered,_=ground_point([(ndc[0]+1)*640,(1-ndc[1])*360],c)
            for actual,expected in zip(recovered,p):self.assertAlmostEqual(actual,expected,places=4)
        c['intrinsics']=None;self.assertEqual(ground_point([10,10],c),(None,'intrinsics_unavailable'))
    def test_ground_nonfinite_rejected(self):
        with self.assertRaises(ValueError):ground_point([float('nan'),1],{})
    def test_abort_is_intentional_and_bounded(self):
        self.assertFalse(abort_due(23,24));self.assertTrue(abort_due(24,24))
        self.assertFalse(abort_due(25,24,returning=True));self.assertFalse(abort_due(25,24,negative_control=True))
        with self.assertRaises(ValueError):abort_due(0,0)
    def test_settling_requires_consecutive_samples(self):
        self.assertIsNone(confirm([(i,i!=5) for i in range(15)]))
        self.assertEqual(confirm([(i,i>=5) for i in range(16)]),15)
    def test_command_without_deceleration_not_event(self):
        self.assertEqual(braking_window([dict(step=i,speed_kmh=30.) for i in range(30)],5),(None,None,False))
    def test_braking_onset_and_censoring(self):
        samples=[dict(step=i,speed_kmh=30. if i<5 else 30.-(i-4)) for i in range(20)]
        self.assertEqual(braking_window(samples,5),(5,19,True))
    def event(self):
        e={k:{} for k in schema()['required']}
        e.update(schema_version='aad-event-0.2-proposed',event_id='e',scenario_id='s',family='A',subject_role='subject',
            target_role='target',surrounding_roles=[],actor_scope='simulator_reference_roles_only',start_step=10,end_step=20,
            dt_s=.1,start_s=1.,end_s=2.,censored=False,physical_behaviour='detected',camera_evidence=[],
            pcds_candidate='unassessed',abnormality_assessment='unassessed',approval='proposed_pending_supervisor_review')
        return e
    def test_event_contract(self):self.assertTrue(validate(self.event(),['subject','target']))
    def test_reversed_interval_and_missing_actor_rejected(self):
        e=self.event();e['end_step']=9
        with self.assertRaises(ValueError):validate(e,['subject','target'])
        with self.assertRaises(ValueError):validate(self.event(),['subject'])
    def test_cannot_promote_abnormality(self):
        e=self.event();e['abnormality_assessment']='abnormal'
        with self.assertRaises(AssertionError):validate(e,['subject','target'])
    def test_unknown_boundaries_not_fabricated(self):
        e=self.event();e.update(start_step=None,end_step=None,start_s=None,end_s=None,physical_behaviour='not_observed')
        self.assertTrue(validate(e,['subject','target']))
        e['physical_behaviour']='detected'
        with self.assertRaises(ValueError):validate(e,['subject','target'])
    def test_step_time_mismatch(self):
        e=self.event();e['start_s']=1.1
        with self.assertRaises(ValueError):validate(e,['subject','target'])
    def test_state_cannot_enter_camera_inference_manifest(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp);(p/'observations.jsonl').write_text(json.dumps(dict(camera_id='c',step=0,time_s=0,file='a.png',image_sha256='x',actor_id='secret')))
            with self.assertRaises(ValueError):list(observations(p))
    def test_proxy_provenance_and_dimensions(self):
        p=json.loads((Path(__file__).parent/'configs/box_proxy.json').read_text());self.assertEqual(vehicle_config(p)['length'],5.74)
        p['length']=-1
        with self.assertRaises(ValueError):vehicle_config(p)
    def test_human_review_requires_identity_and_timestamp(self):
        r=dict(case_id='case',reviewer='',review_complete=False,reviewed_at_utc=None,
            vehicle_present=None,vehicle_class=None,already_counted=None,too_ambiguous=None)
        data=dict(schema_version='vehicle-case-review-0.2',cases_sha256='hash',responses=[r])
        self.assertEqual(validate_responses(data,'hash',['case'])['completed'],0)
        r.update(review_complete=True,vehicle_present='yes',vehicle_class='car',already_counted='no',too_ambiguous='no')
        with self.assertRaises(ValueError):validate_responses(data,'hash',['case'])
        r.update(reviewer='unit_test_fixture_not_a_human_review',reviewed_at_utc='2026-11-19T00:00:00Z')
        self.assertEqual(validate_responses(data,'hash',['case'])['completed'],1)
    def test_review_duplicate_and_foreign_case_rejected(self):
        data=dict(schema_version='vehicle-case-review-0.2',cases_sha256='hash',responses=[dict(case_id='wrong')])
        with self.assertRaises(ValueError):validate_responses(data,'hash',['case'])
    def test_projection_missing_and_behind_camera_preserved(self):
        from .evaluate import project_actor
        a=dict(footprint_components=[dict(world_vertices_m=None)])
        self.assertEqual(project_actor(a,{})['reason'],'unsupported_geometry')
        a['footprint_components'][0]['world_vertices_m']=[[0,-1,0]]*8
        c=dict(world_to_camera_matrix=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],near_m=.1)
        self.assertEqual(project_actor(a,c)['reason'],'near_plane_straddling_or_behind')

    def test_camera_review_requires_contiguous_evidence(self):
        from aad_system.vision import sha
        from .temporal import temporal

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            camera, vision = root/'camera', root/'vision'
            camera.mkdir()
            vision.mkdir()
            calibration = camera/'calibration.json'
            calibration.write_text(json.dumps(dict(width=100,height=100)))
            manifest = camera/'observations.jsonl'
            frames = [dict(step=i,time_s=i*.1,image_sha256=str(i)) for i in range(7)]
            manifest.write_text('\n'.join(map(json.dumps,frames)))
            (vision/'report.json').write_text(json.dumps(dict(source_manifest_sha256=sha(manifest),
                calibration_sha256=sha(calibration))))
            for missing in (False,True):
                for i,(frame,y) in enumerate(zip(frames,(-2,-2,-1,0,1,2,2))):
                    frame['tracks'] = [dict(box_xyxy=[1,1,5,5],class_name='car',track_id=2,
                        estimated_ground_contact_world_m=[18-i*2,0,0])]
                    if not (missing and i==3):
                        frame['tracks'].append(dict(box_xyxy=[6,6,10,10],class_name='car',
                            track_id=1,estimated_ground_contact_world_m=[0,y,0]))
                (vision/'camera_observations.jsonl').write_text('\n'.join(map(json.dumps,frames)))
                output = root/str(missing)
                temporal(camera,vision,output,lane_divider_y=0)
                evidence = json.loads((output/'evidence.json').read_text())
                if missing:
                    self.assertEqual(evidence['events'],[])
                    track = next(t for t in evidence['tracks'] if t['track_id']==1)
                    self.assertEqual(track['missing_captures_between_first_and_last'],1)
                    self.assertIsNone(track['samples'][3]['velocity_xy_m_s'])
                else:
                    event, = evidence['events']
                    self.assertEqual(event['review_status'],'candidate_for_review')
                    self.assertEqual(event['abnormality_assessment'],'unassessed')


if __name__=='__main__':unittest.main()
