import unittest

from .video_tags import tag_tracks
from .roadside_video_tags import in_road_strip


class VideoTagTests(unittest.TestCase):
    def frames(self, context=False, gap=False):
        result=[]
        for frame,y in enumerate((3.5,3.5,3.5,4.5,5.2,5.9,6.5,7.0,7.0)):
            tracks=[] if gap and frame==4 else [dict(track_id=1,xy_m=[frame,y])]
            if context:
                tracks.append(dict(track_id=2,xy_m=[frame-3,7]))
            result.append(tracks)
        return result

    def test_same_maneuver_different_context(self):
        simple,=tag_tracks(self.frames())
        concurrent,=tag_tracks(self.frames(context=True))
        self.assertEqual(simple['tag'],'lane_change_only')
        self.assertEqual(concurrent['tag'],'candidate_AAD_review')
        self.assertEqual(concurrent['interaction_evidence'][0]['track_id'],2)
        self.assertEqual(concurrent['abnormality_assessment'],'unassessed')

    def test_does_not_bridge_missing_observation(self):
        self.assertEqual(tag_tracks(self.frames(context=True,gap=True)),[])

    def test_stationary_off_road_patch_cannot_trigger_flag(self):
        frames=self.frames()
        for tracks in frames:
            tracks.append(dict(track_id=2,xy_m=[4,22]))
        tag,=tag_tracks([[t for t in tracks if in_road_strip(t)] for tracks in frames])
        self.assertEqual(tag['tag'],'lane_change_only')
