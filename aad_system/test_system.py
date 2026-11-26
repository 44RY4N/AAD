import unittest
from pathlib import Path
from .vision import Tracker,iou,Detector
from .__main__ import validate_config


def detection(x=0,c=2):return dict(box_xyxy=[x,0,x+10,10],class_id=c,class_name='car',confidence=.9)


class SystemTests(unittest.TestCase):
    def test_iou(self):self.assertEqual(iou([0,0,10,10],[0,0,10,10]),1)
    def test_continue_and_velocity(self):
        t=Tracker();a=t.update([detection()],0,0);b=t.update([detection(1)],1,.1)
        self.assertEqual(a[0]['track_id'],b[0]['track_id']);self.assertEqual(b[0]['image_velocity_px_s'],[10.,0.])
    def test_missing_and_expiry(self):
        t=Tracker();t.update([detection()],0,0);self.assertEqual(t.update([],1,.1),[])
        self.assertEqual(t.update([detection()],2,.2)[0]['track_id'],1)
        self.assertEqual(t.update([detection()],9,.9)[0]['track_id'],2)
    def test_class_mismatch(self):
        t=Tracker();t.update([detection()],0,0);self.assertEqual(t.update([detection(c=5)],1,.1)[0]['track_id'],2)
    def test_missing_weights(self):
        with self.assertRaises(FileNotFoundError):Detector(Path('not_a_model.onnx'))
    def test_invalid_speed(self):
        for speed in [0,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):validate_config(dict(subject_speed_kmh=speed,leader_speed_kmh=30,target_vehicle_speed_kmh=30,steps=10))
    def test_valid_config(self):validate_config(dict(subject_speed_kmh=30,leader_speed_kmh=30,target_vehicle_speed_kmh=30,steps=10))
    def test_bad_box_and_time(self):
        for box in [[0,0,0,1],[0,0,float('nan'),1]]:
            with self.assertRaises(ValueError):Tracker().update([dict(detection(),box_xyxy=box)],0,0)
        t=Tracker();t.update([],0,0)
        with self.assertRaises(ValueError):t.update([],0,0)


if __name__=='__main__':unittest.main()
