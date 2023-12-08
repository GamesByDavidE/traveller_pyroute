import math


class DistanceOracle:
    def __init__(self, graph):
        self._arcs = [
            [(v, graph.get_edge_data(u, v)["weight"]) for v in graph.neighbors(u)]
            for u in range(len(graph.nodes()))
        ]

    def find_shortest_path(self, s, t, potentials_list):
        arcs = self._arcs
        potentials = max(potentials_list, key=lambda p: abs(p[s] - p[t]))
        swap = potentials[t] > potentials[s]
        if swap:
            s, t = t, s
        distances = [math.inf] * len(self._arcs)
        distances[s] = 0
        parents = [None] * len(self._arcs)
        buckets = [[(0, s)]]
        i = 0
        while i < len(buckets):
            if distances[t] <= i:
                break
            for distance_u, u in buckets[i]:
                if distance_u != distances[u]:
                    continue
                base = distance_u - potentials[u]
                for v, weight in arcs[u]:
                    distance_v = base + weight + potentials[v]
                    if distances[v] <= distance_v:
                        continue
                    distances[v] = distance_v
                    parents[v] = u
                    j = int(max(distance_v, 0))
                    while len(buckets) <= j:
                        buckets.append([])
                    buckets[j].append((distance_v, v))
            i += 1
        if distances[t] == math.inf:
            return None
        path = [t]
        u = t
        while u != s:
            u = parents[u]
            path.append(u)
        if not swap:
            path.reverse()
        return path

    def lighten_edge(self, u, v, weight):
        self._lighten_arc(u, v, weight)
        self._lighten_arc(v, u, weight)

    def _lighten_arc(self, u, v, weight):
        arcs = self._arcs[u]
        for i, (x, _) in enumerate(arcs):
            if x == v:
                arcs[i] = (v, weight)
                break
        else:
            assert False
