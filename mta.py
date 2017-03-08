import sys
import transitfeed

DATA_FILENAME = './data/google_transit.zip'
SERVICE_PERIODS = [
    u"A20161106WKD",
    u"B20161106WKD",
    u"R20161106WKD"
]

def time(time_string):
    time_string = time_string.replace(':', '')
    return int(time_string)

class Edge(object):
    def __init__(self, origin, destination):
        self.origin = origin
        self.destination = destination
        self.weight = time(destination.arrival_time) - time(origin.departure_time)


def build_edges_from_trip(trip):
    stop_times = trip.GetStopTimes()

    edges = []
    # stop_times seems to be sorted by sequence_num already
    for i in range(len(stop_times) - 1):
        time = stop_times[i]
        next_time = stop_times[i+1]

        edge = Edge(time, next_time)
        edges.append(edge)

    return edges

def get_edges_with_schedule(schedule):
    trips = [t for t in schedule.trips.values() if t.service_id in SERVICE_PERIODS]
    edges_per_trip = [build_edges_from_trip(trip) for trip in trips]

    es = [e for edges in edges_per_trip for e in edges]
    return es

def get_edges_for_stop(all_edges, stop):
    es = []
    i = 0
    while i < len(all_edges):
        edge = all_edges[i]
        if edge.origin.stop_id == stop.stop_id:
            es.append(edge)
            all_edges.pop(i)
        else:
            i+=1
    return es

def get_parent_stop_id(stop_id):
    if stop_id[-1] in ('N', 'S'):
        return stop_id[0:-1]
    else:
        return stop_id

def getSchedule():
    schedule = transitfeed.Schedule()
    schedule.Load(DATA_FILENAME)
    return schedule

def build_edge_index(stops, edges):
    stops_and_edges = {}

    for stop in stops:
        parent_stop = get_parent_stop_id(stop.stop_id)
        if stops_and_edges.has_key(parent_stop):
            stops_and_edges[parent_stop] += get_edges_for_stop(edges, stop)
        else:
            stops_and_edges[parent_stop] = get_edges_for_stop(edges, stop)

    print len(stops_and_edges)
    return stops_and_edges

def isChildStop(stop):
    return stop.stop_id[-1] in (u'N', u'S')

def main():
    s = getSchedule()

    stops = s.stops.values()
    edges = get_edges_with_schedule(s)
    stop_edge_map = build_edge_index(stops, edges)
    nodes = [stop for stop in stops if not isChildStop(stop) ]

    return s, nodes, stop_edge_map

def dijkstra(stop_ids, stop_edge_map, source):
    dist = {}
    prev = {}
    Q = []

    dist[source] = 0
    prev[source] = None
    for stop_id in stop_ids:
        if not stop_id == source:
            dist[stop_id] = sys.maxint     # infinity
            prev[stop_id] = None
        Q.append((stop_id, dist[stop_id]))

    sorted_Q = Q        #will sort later
    while len(sorted_Q) > 0:
        sorted_Q = sorted(sorted_Q, key=lambda x: x[1])
        curr = sorted_Q.pop(0)[0]

        for edge in stop_edge_map[curr]:
            neighbor = get_parent_stop_id(edge.destination.stop_id)
            alt = dist[curr] + edge.weight
            if alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = curr

                i = [i for i, t in enumerate(sorted_Q) if t[0] == neighbor][0]
                sorted_Q[i] = (neighbor, alt)

    return dist, prev
        
def path_to_nearest_unvisited_stop(nodes, stop_edge_map, visited, source):
    stop_ids = [node.stop_id for node in nodes]
    dist, prev = dijkstra(stop_ids, stop_edge_map, source)

    unvisited = [stop_id for stop_id in stop_ids if not stop_id in visited]
    min_unvisited = unvisited[0]
    min_dist = sys.maxint
    for stop in unvisited:
        if dist[stop] < min_dist:
            min_unvisited = stop
            min_dist = dist[stop]

    curr = min_unvisited
    path = [curr]
    while prev[curr]:
        curr = prev[curr]
        path.insert(0, curr)
        
    return path

def _dfs(node, stop_edge_map, visited):
    print node
    visited.append(node)
    current_edges = sorted(stop_edge_map[node], key=lambda edge: edge.weight)

    for neighbor in current_edges:
        stop = get_parent_stop_id(neighbor.destination.stop_id)
        if not stop in visited:
            visited = _dfs(stop, stop_edge_map, visited)

    return visited
    
def dfs(nodes, stop_edge_map):
    visited = []

    while len(visited) < len(nodes):
        stop = [node.stop_id for node in nodes if not node.stop_id in visited][0]
        visited = _dfs(stop, stop_edge_map, visited) 

    return visited

def nn(s, nodes, stop_edge_map):
    current = nodes[0].stop_id
    visited = [current]
    path = [current]

    while (len(visited) < len(nodes)):
        current_edges = sorted(stop_edge_map[current], key=lambda edge: edge.weight)
        unvisited_edges = [edge for edge in current_edges if not get_parent_stop_id(edge.destination.stop_id) in visited]

        if (len(unvisited_edges) > 0):
            shortest_edge = unvisited_edges[0]
            current = get_parent_stop_id(shortest_edge.destination.stop_id)
        else:
            path_to_nearest = path_to_nearest_unvisited_stop(nodes,
                                 stop_edge_map, visited, 
                                 current)
            current = path_to_nearest.pop()
            path += path_to_nearest

        visited.append(current)
        path.append(current)

    return path


if __name__ == '__main__':
    import time
    start_time = time.time()
    s, nodes, stop_edge_map = main()
    nn(s, nodes, stop_edge_map) 
    print("--- %s seconds ---" % (time.time() - start_time))
