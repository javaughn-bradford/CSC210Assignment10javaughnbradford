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
    
    def path_map_to_path(self, previous_map, goal):
        path = []
        current = goal
        while True:
            path.insert(0, current)
            previous = previous_map[current]
            if previous == current:
                break
            current = previous
        return path

    # dijkstra's
    def dijkstra(self, start):
        # parents keeps track of how we reached each node
        parents = {start: start}
        distances = {start: 0}

        pq = PriorityQueue()
        pq.put((0, start))   #distance

        # Loop until no more nodes are left to process
        while not pq.empty():
            current_dist, current = pq.get()

            for edge in self.edges_from(current):
                new_distance = current_dist + edge.weight

                if edge.to not in distances or new_distance < distances[edge.to]:
                    distances[edge.to] = new_distance
                    parents[edge.to] = current
                    pq.put((new_distance, edge.to))

        return parents, distances
    
    
    def __str__(self):
        return '\n'.join(
            f"{vertex}: {[f'{edge.to}({edge.weight})' for edge in edges]}"
            for vertex, edges in self._adjacency_list.items()
        )
