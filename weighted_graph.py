# weighted_graph.py
# A weighted graph stored using an adjacency list
# Modified by:
# Note: Please write this yourself, not using an LLM.
from queue import PriorityQueue
from dataclasses import dataclass

@dataclass(order=True, frozen=True)
class WeightedEdge:
    weight: float | int
    from_: str
    to: str 


class WeightedGraph:

    def __init__(self):
        self._adjacency_list = {}
        
    def add_vertex(self, vertex):
        if vertex not in self._adjacency_list:
            self._adjacency_list[vertex] = set()
            
    def add_edge(self, from_, to, weight, bidirectional=True):
        if from_ not in self._adjacency_list:
            self._adjacency_list[from_] = set()
        self._adjacency_list[from_].add(WeightedEdge(weight, from_, to))
        if bidirectional:
            if to not in self._adjacency_list:
                self._adjacency_list[to] = set()
            self._adjacency_list[to].add(WeightedEdge(weight, to, from_))
        else:
            if to not in self._adjacency_list:
                self.add_vertex(to)
                
    def neighbors(self, vertex):
        assert vertex in self._adjacency_list, "Vertex not in graph"
        return {edge.to for edge in self._adjacency_list[vertex]}
    
    def edges_from(self, vertex):
        assert vertex in self._adjacency_list, "Vertex not in graph"
        return self._adjacency_list[vertex]
    
    def edge_exists(self, from_, to):
        if from_ not in self._adjacency_list:
            return False
        return to in self.neighbors(from_)

    # Perform jarnik's algorithm from *start*, looking through
    # the entire graph to form a minimum-spanning-tree (MST)
    # Returns a list of weighted edges composing the MST
    def jarnik(self, start):
        result = [] # final weighted edges in MST
        # YOUR CODE HERE
        
        return result
    
    
    def __str__(self):
        return '\n'.join(f"{vertex}: {[f'{edge.to}({edge.weight})' for edge in edges]}"
                         for vertex, edges in self._adjacency_list.items())