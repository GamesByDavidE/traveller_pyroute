"""
Created on Sep 23, 2023

@author: CyberiaResurrection

Thanks to @GamesByDavidE for original prototype and design discussions
"""
import math

import numpy as np


class DistanceGraph:

    def __init__(self, graph):
        rawnodes = graph.nodes()
        self._nodes = list(graph.nodes())
        self._indexes = {node: i for (i, node) in enumerate(self._nodes)}
        self._arcs = [
            (np.array(graph.adj[u]), np.array([data['weight'] for data in list(graph.adj[u].values())], dtype=float))
            for u in self._nodes
        ]
        self._locations = [
            np.array([rawnodes[node]['star'].x, rawnodes[node]['star'].y, rawnodes[node]['star'].z]) for node in graph.nodes
        ]


    def lighten_edge(self, u, v, weight):
        self._lighten_arc(u, v, weight)
        self._lighten_arc(v, u, weight)

    def _lighten_arc(self, u, v, weight):
        arcs = self._arcs[u]
        flip = arcs[0] == v
        if flip.any():
            arcs[1][flip] = weight
        else:
            assert False

    def find_shortest_path(self, s, t):
        numpy_abs = np.abs
        numpy_max = np.max
        arcs = self._arcs
        locations = self._locations
        s = self._indexes[s]
        t = self._indexes[t]
        distances = np.ones(len(self._nodes)) * math.inf
        heuristics = np.zeros(len(self._nodes))
        upper_bound = math.inf
        distances[s] = 0
        parents = [None] * len(self._nodes)
        location_t = locations[t]

        heuristic_s = numpy_max(numpy_abs(locations[s] - location_t))
        buckets = [[(0, s, heuristic_s)]]
        found_t = False
        for bucket in buckets:
            for distance_u, u, heuristic_u in bucket:
                if distance_u != distances[u] or upper_bound < distance_u:
                    continue
                if u == t:
                    found_t = True
                base = distance_u - heuristic_u

                neighbours = arcs[u][0]
                weights = arcs[u][1] + base
                adjusted_weights = weights + heuristics[neighbours]

                keep = np.logical_and(adjusted_weights < distances[neighbours], adjusted_weights <= upper_bound)
                neighbours = neighbours[keep]
                weights = weights[keep]

                max_index = len(neighbours)

                for i in range(0, max_index):
                    v = neighbours[i]
                    weight = weights[i]
                    delta = locations[v] - location_t
                    heuristic_v = numpy_max(np.maximum(delta, -1 * delta))
                    heuristics[v] = heuristic_v
                    distance_v = (
                            weight
                            + heuristic_v
                    )
                    if distances[v] <= distance_v or upper_bound < distance_v:
                        continue
                    distances[v] = distance_v
                    parents[v] = u
                    if v == t:
                        upper_bound = distance_v
                    intdist = int(distance_v)
                    while len(buckets) <= intdist:
                        buckets.append([])
                    buckets[intdist].append((distance_v, v, heuristic_v))
            if found_t:
                break
        else:
            return None
        path = []
        u = t
        while u is not None:
            path.append(u)
            u = parents[u]
        return [self._nodes[i] for i in reversed(path)]
