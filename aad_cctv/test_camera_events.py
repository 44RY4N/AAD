"""Fixtures for conservative camera-event evidence gates."""
import unittest

from .camera_events import crossings, decelerations, passes


def sample(i, x, y):
    return dict(step=i, capture_index=i, time_s=i*.1, xy_m=[x,y])


class CameraEventTests(unittest.TestCase):
    def test_deceleration_requires_consecutive_contact_estimates(self):
        samples=[sample(i,i-.025*i*i,0) for i in range(11)]
        event,=decelerations({1:samples},[])
        self.assertEqual(event['family'],'C')
        self.assertEqual(event['review_status'],'insufficient_review_evidence')
        self.assertEqual(list(decelerations({1:samples[:6]+samples[7:]},[])),[])

    def test_pass_requires_return_and_same_counterpart_reversal(self):
        changes=[dict(track_id=1,start_step=0,end_step=3),dict(track_id=1,start_step=7,end_step=10)]
        subject=[sample(i,2*i,2 if 3<=i<=7 else -2) for i in range(11)]
        counterpart=[sample(i,5+i,-2) for i in range(11)]
        event,=passes(changes,{1:subject,2:counterpart},0)
        self.assertEqual(event['family'],'B')
        self.assertEqual(event['interaction_evidence']['counterpart_track'],2)
        self.assertEqual(list(passes(changes,{1:subject,2:counterpart[:-1]},0)),[])

    def test_crossing_requires_two_consecutive_samples_and_deduplicates(self):
        a=[sample(i,.2,.5) for i in range(4)]
        b=[sample(i,.8,.5) for i in range(4)]
        event,=crossings({1:a,2:b},[0,0,1,1])
        self.assertEqual(event['review_status'],'candidate_for_review')
        self.assertEqual(event['abnormality_assessment'],'unassessed')
        self.assertEqual(list(crossings({1:a,2:b[::2]},[0,0,1,1])),[])
