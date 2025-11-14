# tests.py
# Tests for your WeightedGraph.jarnik()
# Do not modify the provided tests
import unittest
from weighted_graph import WeightedGraph

def print_edges(wes):
    for we in wes:
        print(f"({we.from_} -{we.weight}> {we.to}), ", end="")
    print()
    
def total_weight(wes):
    return sum(we.weight for we in wes)

# DO NOT MODIFY THIS TEST CASE
class TestMSTCityGraph1(unittest.TestCase):
    def test_mst_city_graph_1(self):
        cityGraph1 = WeightedGraph()
        cityGraph1.add_edge("Seattle", "Chicago", 2097)
        cityGraph1.add_edge("Seattle", "Denver", 1331)
        cityGraph1.add_edge("Seattle", "San Francisco", 807)
        cityGraph1.add_edge("San Francisco", "Denver", 1267)
        cityGraph1.add_edge("San Francisco", "Los Angeles", 381)
        cityGraph1.add_edge("Los Angeles", "Denver", 1015)
        cityGraph1.add_edge("Los Angeles", "Kansas City", 1663)
        cityGraph1.add_edge("Los Angeles", "Dallas", 1435)
        cityGraph1.add_edge("Denver", "Chicago", 1003)
        cityGraph1.add_edge("Denver", "Kansas City", 599)
        cityGraph1.add_edge("Kansas City", "Chicago", 533)
        cityGraph1.add_edge("Kansas City", "New York", 1260)
        cityGraph1.add_edge("Kansas City", "Atlanta", 864)
        cityGraph1.add_edge("Kansas City", "Dallas", 496)
        cityGraph1.add_edge("Chicago", "Boston", 983)
        cityGraph1.add_edge("Chicago", "New York", 787)
        cityGraph1.add_edge("Boston", "New York", 214)
        cityGraph1.add_edge("Atlanta", "New York", 888)
        cityGraph1.add_edge("Atlanta", "Dallas", 781)
        cityGraph1.add_edge("Atlanta", "Houston", 810)
        cityGraph1.add_edge("Atlanta", "Miami", 661)
        cityGraph1.add_edge("Houston", "Miami", 1187)
        cityGraph1.add_edge("Houston", "Dallas", 239)
        
        print("------cityGraph1------")
        print(cityGraph1)
        
        result = cityGraph1.jarnik("New York")
        
        print("------cityGraph1 mst------")
        print_edges(result)
        
        # Are there the right number of edges in the result?
        self.assertEqual(len(result), len(cityGraph1._adjacency_list) - 1)
        # Is it the right total weight?
        self.assertEqual(total_weight(result), 6513)
        # Make sure these are all valid edges
        for we in result:
            self.assertTrue(cityGraph1.edge_exists(we.from_, we.to))
            
# DO NOT MODIFY THIS TEST CASE
class TestMSTCityGraph2(unittest.TestCase):
    def test_mst_city_graph_2(self):
        cityGraph2 = WeightedGraph()
        cityGraph2.add_edge("Seattle", "Chicago", 1737)
        cityGraph2.add_edge("Seattle", "San Francisco", 678)
        cityGraph2.add_edge("San Francisco", "Riverside", 386)
        cityGraph2.add_edge("San Francisco", "Los Angeles", 348)
        cityGraph2.add_edge("Los Angeles", "Riverside", 50)
        cityGraph2.add_edge("Los Angeles", "Phoenix", 357)
        cityGraph2.add_edge("Riverside", "Phoenix", 307)
        cityGraph2.add_edge("Riverside", "Chicago", 1704)
        cityGraph2.add_edge("Phoenix", "Dallas", 887)
        cityGraph2.add_edge("Phoenix", "Houston", 1015)
        cityGraph2.add_edge("Dallas", "Chicago", 805)
        cityGraph2.add_edge("Dallas", "Atlanta", 721)
        cityGraph2.add_edge("Dallas", "Houston", 225)
        cityGraph2.add_edge("Houston", "Atlanta", 702)
        cityGraph2.add_edge("Houston", "Miami", 968)
        cityGraph2.add_edge("Atlanta", "Chicago", 588)
        cityGraph2.add_edge("Atlanta", "Washington", 543)
        cityGraph2.add_edge("Atlanta", "Miami", 604)
        cityGraph2.add_edge("Miami", "Washington", 923)
        cityGraph2.add_edge("Chicago", "Detroit", 238)
        cityGraph2.add_edge("Detroit", "Boston", 613)
        cityGraph2.add_edge("Detroit", "Washington", 396)
        cityGraph2.add_edge("Detroit", "New York", 482)
        cityGraph2.add_edge("Boston", "New York", 190)
        cityGraph2.add_edge("New York", "Philadelphia", 81)
        cityGraph2.add_edge("Philadelphia", "Washington", 123)
        
        print("------cityGraph2------")
        print(cityGraph2)
        
        result = cityGraph2.jarnik("New York")
        
        print("------cityGraph2 mst------")
        print_edges(result)
        
        # Are there the right number of edges in the result?
        self.assertEqual(len(result), len(cityGraph2._adjacency_list) - 1)
        # Is it the right total weight?
        self.assertEqual(total_weight(result), 5372)
        # Make sure these are all valid edges
        for we in result:
            self.assertTrue(cityGraph2.edge_exists(we.from_, we.to))

            
if __name__ == "__main__":
    unittest.main()
    