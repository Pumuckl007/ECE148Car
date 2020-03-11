import unittest

from graph import Graph

class TestGraphMethods(unittest.TestCase):

    def test_distance(self):
        graph = Graph()
        start = (32.880891, -117.235142)
        end = (32.881131, -117.235242)
        start = graph.point_to_rads(start)
        end = graph.point_to_rads(end)
        distance = graph.dist_between_gps_points(start, end)
        self.assertLess(abs(distance - 28.14), 0.2)

    def test_closest_node(self):
        graph = Graph()
        start = (32.880891, -117.235142)
        end = (32.881131, -117.235242)
        graph.add_node(start)
        graph.add_node(end)
        query = (32.880915, -117.235328)
        query = graph.point_to_rads(query)
        index = graph.find_closest_vertex(query)
        self.assertEqual(index, 0)

    def test_find_path_simple(self):
        graph = Graph()
        n1 = (32.880891, -117.235142)
        n2 = (32.881008, -117.235226)
        n3 = (32.881131, -117.235242)
        idx = graph.add_node(n1)
        idx = graph.append_node(n2, idx)
        graph.append_node(n3, idx)
        start = (32.881152, -117.235134)
        end = (32.880859, -117.235241)
        path = graph.find_path(start, end)
        self.assertAlmostEqual(path[0][0], n3[0], places=9)
        self.assertAlmostEqual(path[0][1], n3[1], places=9)
        self.assertAlmostEqual(path[1][0], n2[0], places=9)
        self.assertAlmostEqual(path[1][1], n2[1], places=9)
        self.assertAlmostEqual(path[2][0], n1[0], places=9)
        self.assertAlmostEqual(path[2][1], n1[1], places=9)

    def test_find_path_branch(self):
        graph = Graph()
        n1 = (32.880891, -117.235142)
        n2 = (32.881008, -117.235226)
        n3 = (32.881131, -117.235242)
        n4 = (32.880904, -117.235373)
        idx = graph.add_node(n1)
        idx = graph.append_node(n2, idx)
        graph.append_node(n3, idx)
        graph.append_node(n4, idx)
        start = (32.881152, -117.235134)
        end = (32.880859, -117.235241)
        path = graph.find_path(start, end)
        self.assertAlmostEqual(path[0][0], n3[0], places=9)
        self.assertAlmostEqual(path[0][1], n3[1], places=9)
        self.assertAlmostEqual(path[1][0], n2[0], places=9)
        self.assertAlmostEqual(path[1][1], n2[1], places=9)
        self.assertAlmostEqual(path[2][0], n1[0], places=9)
        self.assertAlmostEqual(path[2][1], n1[1], places=9)

    def test_find_path_branch_stop_early(self):
        graph = Graph()
        n1 = (32.880891, -117.235142)
        n2 = (32.881008, -117.235226)
        n3 = (32.881131, -117.235242)
        n4 = (32.880904, -117.235373)
        n5 = (32.880890, -117.235578)
        idx = graph.add_node(n1)
        idx = graph.append_node(n2, idx)
        graph.append_node(n3, idx)
        idx = graph.append_node(n4, idx)
        idx = graph.append_node(n5, idx)
        start = (32.881152, -117.235134)
        end = (32.880936, -117.235396)
        path = graph.find_path(start, end)
        self.assertAlmostEqual(path[0][0], n3[0], places=9)
        self.assertAlmostEqual(path[0][1], n3[1], places=9)
        self.assertAlmostEqual(path[1][0], n2[0], places=9)
        self.assertAlmostEqual(path[1][1], n2[1], places=9)
        self.assertAlmostEqual(path[2][0], n4[0], places=9)
        self.assertAlmostEqual(path[2][1], n4[1], places=9)

if __name__ == '__main__':
    unittest.main()
