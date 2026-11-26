import unittest

from .video_event_families import braking, crossing, overtaking


class MultiFamilyVideoTests(unittest.TestCase):
    def test_pass_requires_a_return_and_order_reversal(self):
        changes=[dict(track_id=1,start_frame=4,end_frame=7),dict(track_id=1,start_frame=21,end_frame=24)]
        ego={i:(i,7 if 7<=i<=21 else 3.5) for i in range(25)}
        lead={i:(9+.35*i,3.5) for i in range(25)}
        tag,=overtaking(changes,{1:ego,2:lead})
        self.assertEqual(tag['tag'],'pass_and_return')
        self.assertLess(tag['interaction_evidence']['final_longitudinal_order_m'],0)
        self.assertEqual(list(overtaking(changes[:1],{1:ego,2:lead})),[])

    def test_speed_drop_and_closing_leader_are_distinct(self):
        subject={i:(.8*min(i,8)+.4*max(i-8,0),3.5) for i in range(13)}
        lead={i:(12+.2*i,3.5) for i in range(13)}
        event,=braking({1:subject},10)
        self.assertEqual(event['tag'],'deceleration')
        event,=braking({1:subject,2:lead},10)
        self.assertEqual(event['tag'],'candidate_AAD_review')
        self.assertEqual(event['abnormality_assessment'],'unassessed')

    def test_crossing_gap_changes_the_review_tag(self):
        horizontal={i:(91+i,3.5) for i in range(16)}
        for offset,tag in ((2,'candidate_AAD_review'),(20,'crossing_interaction')):
            vertical={i+offset:(97,i-2) for i in range(16)}
            event,=crossing({1:horizontal,2:vertical},10)
            self.assertEqual(event['tag'],tag)
