import copy
import math
import unittest
from shapely.geometry import Polygon

from .polygons import actor_separation,actor_ttc,occupancy,polygon,separation,swept_ttc,union_region


def box(x=0,y=0,hx=1,hy=1,angle=0):
    c,s=math.cos(angle),math.sin(angle)
    return [[x+c*a-s*b,y+s*a+c*b] for a,b in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]


class SeparationTests(unittest.TestCase):
    def test_known_axis_distance_and_closest_points(self):
        result=separation(box(),box(x=5))
        self.assertEqual(result["distance"],3)
        self.assertEqual(math.dist(*result["closest_points"]),3)

    def test_translated_distance_invariance(self):
        self.assertEqual(separation(box(100,-30),box(105,-30))["distance"],3)

    def test_rotated_rectangles(self):
        result=separation(box(),box(x=5,angle=math.pi/4))
        self.assertAlmostEqual(result["distance"],4-math.sqrt(2))
        self.assertAlmostEqual(result["distance"],Polygon(box()).distance(Polygon(box(x=5,angle=math.pi/4))))

    def test_touching_distinct_from_overlap(self):
        self.assertEqual(separation(box(),box(x=2))["relation"],"touching")
        result=separation(box(),box(x=1.99))
        self.assertEqual(result["relation"],"overlapping");self.assertEqual(result["distance"],0)
        self.assertIsNone(result["penetration_depth"])

    def test_shape_offsets_and_different_orientations(self):
        # Reference B at x=5 with actual centre offset -1 => gap 2, not 3.
        self.assertEqual(separation(box(),box(x=4))["distance"],2)
        a,b=box(angle=.21),box(x=4,y=.7,angle=-.43)
        self.assertAlmostEqual(separation(a,b)["distance"],Polygon(a).distance(Polygon(b)),places=10)

    def test_disconnected_components_are_not_convexly_filled(self):
        components={0:box(x=-5),1:box(x=5)}
        result=actor_separation(components,{7:box()})
        self.assertEqual(result["distance"],3)
        self.assertEqual(union_region(components).area,8)
        self.assertEqual(len(result["component_pairs"]),2)

    def test_incomplete_components_fail(self):
        with self.assertRaises(ValueError):actor_separation({0:box(),1:None},{0:box(x=5)})
        with self.assertRaises(ValueError):actor_separation({}, {0:box()})

    def test_sampled_spatial_oracle_grid(self):
        for x,y,angle in [(4,2,.2),(0,0,.5),(2,0,0),(3,-1,.7),(-4,1,.31),(1.2,1.1,-.8)]:
            a,b=box(angle=.1),box(x,y,angle=angle)
            self.assertAlmostEqual(separation(a,b)["distance"],Polygon(a).distance(Polygon(b)),places=9)


class OccupancyTests(unittest.TestCase):
    def test_whole_lane(self):
        r=occupancy({0:box()},box(hx=10,hy=2))
        self.assertEqual((r["actor_area"],r["intersection_area"],r["fraction"],r["state"]),(4,4,1,"whole"))

    def test_straddling_two_lanes(self):
        for centre in [-1,1]:
            self.assertEqual(occupancy({0:box()},box(y=centre,hx=10))["fraction"],.5)

    def test_outside_recorded_road(self):
        r=occupancy({0:box(x=20)},box(hx=10,hy=2))
        self.assertEqual(r["fraction"],0);self.assertEqual(r["state"],"outside")

    def test_partial_boundary(self):
        r=occupancy({0:box(y=1.5)},box(hx=10,hy=1))
        self.assertEqual(r["fraction"],.25);self.assertEqual(r["state"],"partial")

    def test_overlapping_components_counted_once(self):
        r=occupancy({0:box(),1:box(x=1)},box(hx=10,hy=10))
        self.assertEqual(r["actor_area"],6);self.assertEqual(r["fraction"],1)

    def test_disconnected_occupancy_preserves_empty_space(self):
        r=occupancy({0:box(x=-5),1:box(x=5)},box())
        self.assertEqual(r["intersection_area"],0)


class TTCTests(unittest.TestCase):
    def test_closing_same_direction(self):
        self.assertEqual(swept_ttc(box(),box(x=10),[2,0],[0,0],5)["value"],4)

    def test_closing_opposite_velocity_vectors(self):
        self.assertEqual(swept_ttc(box(),box(x=10),[1,0],[-1,0],5)["value"],4)

    def test_crossing(self):
        self.assertEqual(swept_ttc(box(x=-5),box(y=-5),[1,0],[0,1],10)["value"],3)

    def test_moving_apart(self):
        r=swept_ttc(box(),box(x=10),[-1,0],[1,0],5)
        self.assertEqual(r["validity"],"undefined");self.assertIsNone(r["value"])

    def test_current_overlap_and_touch(self):
        for x,word in [(1,"overlapping"),(2,"touching")]:
            r=swept_ttc(box(),box(x=x),[-1,0],[1,0],5)
            self.assertEqual(r["value"],0);self.assertIn(word,r["reason"])

    def test_horizon_censored_and_exact_boundary(self):
        r=swept_ttc(box(),box(x=10),[2,0],[0,0],3)
        self.assertEqual(r["validity"],"censored");self.assertIsNone(r["value"])
        self.assertEqual(swept_ttc(box(),box(x=10),[2,0],[0,0],4)["value"],4)

    def test_parallel_nonintersecting(self):
        self.assertEqual(swept_ttc(box(),box(x=5,y=3),[1,0],[0,0],10)["validity"],"undefined")

    def test_common_velocity_translation_invariance(self):
        self.assertEqual(swept_ttc(box(),box(x=10),[9,-3],[7,-3],5)["value"],4)

    def test_rotated_footprint_future_intersection(self):
        r=swept_ttc(box(),box(x=5,angle=math.pi/4),[1,0],[0,0],5)
        self.assertAlmostEqual(r["value"],4-math.sqrt(2))

    def test_multi_component_first_contact(self):
        r=actor_ttc({0:box(),1:box(x=-5)},{8:box(x=10)},[2,0],[0,0],10)
        self.assertEqual(r["value"],4);self.assertEqual(r["component_ids"],[0,8])

    def test_missing_invalid_velocity(self):
        for v in [None,[1],[float('nan'),0],[float('inf'),0]]:
            r=swept_ttc(box(),box(x=5),v,[0,0],5)
            self.assertIsNone(r["value"]);self.assertEqual(r["validity"],"unavailable")

    def test_invalid_horizon(self):
        for h in [0,-1,float('inf')]:
            with self.assertRaises(ValueError):swept_ttc(box(),box(x=5),[0,0],[0,0],h)

    def test_entry_and_exit_against_independent_geometry(self):
        a,b=box(angle=.13),box(x=7,y=.6,angle=-.31)
        r=swept_ttc(a,b,[2,.1],[0,0],10);t=r["value"]
        self.assertIsNotNone(t)
        move=lambda time:[[p[0]+2*time,p[1]+.1*time] for p in a]
        self.assertGreater(Polygon(move(t-1e-5)).distance(Polygon(b)),0)
        self.assertGreater(Polygon(move(t+1e-5)).intersection(Polygon(b)).area,0)


class NumericalTests(unittest.TestCase):
    def test_duplicate_closing_and_consecutive_vertices(self):
        p=[[-1,-1],[-1,-1],[1,-1],[1,1],[-1,1],[-1,-1]]
        self.assertEqual(Polygon(polygon(p)).area,4)

    def test_collinear_edge_samples(self):
        self.assertEqual(len(polygon([[-1,-1],[0,-1],[1,-1],[1,1],[-1,1]])),4)

    def test_nearly_parallel_edges(self):
        a=box();b=box(x=4,angle=1e-10)
        self.assertAlmostEqual(separation(a,b)["distance"],Polygon(a).distance(Polygon(b)))

    def test_degenerate_malformed_nonfinite(self):
        for p in [[],[[0,0],[1,0],[2,0]],[[0,0],[1,1],[0,1],[1,0]],[[0,0],[1,0],[float('nan'),1]],[[0,0],[1,0],[1,float('inf')]],[[0,0],[1,0],[0,0],[0,1]],[[0],[1,0],[0,1]]]:
            with self.assertRaises(ValueError):polygon(p)

    def test_concave_rejected_not_filled(self):
        with self.assertRaises(ValueError):polygon([[0,0],[2,0],[1,.5],[2,2],[0,2]])


if __name__=="__main__":unittest.main()
