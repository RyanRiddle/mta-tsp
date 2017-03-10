import sys
import heapq
import transitfeed

DATA_FILENAME = './data/google_transit.zip'
SERVICE_PERIODS = [
    u"A20161106WKD",
    u"B20161106WKD",
    u"R20161106WKD"
]
ANYTIME = 999999

def seconds_past_midnight(time_string):
    time_parts = [int(piece) for piece in time_string.split(":")]
    seconds_per_part = [3600, 60, 1]
    return reduce(lambda a, b: a + b, 
                    [a * b for a, b in zip(time_parts, seconds_per_part)])

class Edge(object):
    def __init__(self, origin, destination, depart_at, arrive_at):
        self.origin = origin
        self.destination = destination
        self.depart_at = depart_at
        self.arrive_at = arrive_at

    def get_arrival_time(self, current_time):
        if self.depart_at == ANYTIME:
            return current_time + self.arrive_at
        else:
            return self.arrive_at

    def get_departure_time(self, current_time):
        if self.depart_at == ANYTIME:
            return current_time
        else:
            return self.depart_at

def get_transfers_from_schedule(schedule):
    transfers_ll = schedule._transfers.values()
    return [transfer for transfer_l in transfers_ll 
                     for transfer in transfer_l]

def build_edges_from_transfers(transfers, nodes):
    edges = []
    for transfer in transfers:
        origin      = get_stop(nodes, transfer.from_stop_id) 
        destination = get_stop(nodes, transfer.to_stop_id) 
        edge = Edge(origin, destination, ANYTIME, transfer.min_transfer_time)
        edges.append(edge)
    
    return edges

def build_edges_from_trip(trip, nodes):
    stop_times = trip.GetStopTimes()

    edges = []
    # stop_times is sorted by stop_sequence
    for i in range(len(stop_times) - 1):
        stop_time = stop_times[i]
        origin    = get_stop(nodes, stop_time.stop_id) 
        depart_at = seconds_past_midnight(stop_time.departure_time)

        next_stop_time  = stop_times[i+1]
        destination     = get_stop(nodes, next_stop_time.stop_id)
        arrive_at       = seconds_past_midnight(next_stop_time.arrival_time)

        edge = Edge(origin, destination, depart_at, arrive_at)
        edges.append(edge)

    return edges

def get_edges_with_schedule(schedule, nodes):
    trips = [trip for trip in schedule.trips.values() 
                if trip.service_id in SERVICE_PERIODS]

    edges_per_trip = [build_edges_from_trip(trip, nodes) for trip in trips]

    trip_edges = [edge for edges in edges_per_trip 
                       for edge in edges]

    transfers = get_transfers_from_schedule(schedule) 
    transfer_edges = build_edges_from_transfers(transfers, nodes)

    return trip_edges + transfer_edges

def get_parent_stop_id(stop_id):
    if stop_id[-1] in ('N', 'S'):
        return stop_id[0:-1]
    else:
        return stop_id

def dict_concat(d1, d2):
    d = {}

    for k1 in d1.keys():
        d[k1] = d1[k1]

    for k2 in d2.keys():
        if d.has_key(k2):
            d[k2] += d2[k2]
        else:
            d[k2] = d2[k2]

    return d
        
def prune_stops(stops, edges):
    return [stop for stop in stops if edges.has_key(stop.stop_id) and len(edges[stop.stop_id]) > 0]

def remove_blacklisted(nodes):
    blacklist = [
                'S09',
                'S10',
                'S11',
                'S12',
                'S13',
                'S14',
                'S15',
                'S16',
                'S17',
                'S18',
                'S19',
                'S20',
                'S21',
                'S22',
                'S23',
                'S24',
                'S25',
                'S26',
                'S27',
                'S28',
                'S29',
                'S30',
                'S31',]
    return [node for node in nodes if node.stop_id not in blacklist]

def get_edges_for_stop(all_edges, stop):
    return [edge for edge in all_edges if edge.origin.stop_id == stop.stop_id]

def build_edge_index(stops, all_edges):
    stops_and_edges = {}

    for stop in stops:
        edges   = get_edges_for_stop(all_edges, stop)
        sorted_edges = sorted(edges, key=lambda edge: edge.depart_at)
        stops_and_edges[stop.stop_id] = sorted_edges

    return stops_and_edges

def is_child_stop(stop):
    return stop.stop_id[-1] in ('N', 'S')

def load_schedule():
    schedule = transitfeed.Schedule()
    schedule.Load(DATA_FILENAME)
    return schedule

def get_nodes(schedule):
    stops = schedule.stops.values()
    nodes = [stop for stop in stops if not is_child_stop(stop) ]
    return nodes

def main():
    schedule = load_schedule()
    nodes = get_nodes(schedule)

    edges = get_edges_with_schedule(schedule, nodes)
    stop_edge_map = build_edge_index(nodes, edges)

    #nodes = prune_stops(nodes, stop_edge_map)
    #nodes = remove_blacklisted(nodes)

    return schedule, nodes, stop_edge_map

def get_edges_after_time(edges, time):
    return [edge for edge in edges if edge.depart_at >= time]

def decrease_key(Q, time_and_node):
    # hack to find element 
    indices = [i for i, el in enumerate(Q) if el[1] == time_and_node[1]]
    if len(indices) > 0:
        i = indices[0] 
        Q[i] = time_and_node
    else:
        heapq.heappush(Q, time_and_node)
    
    return Q 

def dijkstra(stop_ids, stop_edge_map, source, source_arrive_time):
    INFINITY = sys.maxint
    arrival_time = {}
    prev = {}
    Q = []

    arrival_time[source] = source_arrive_time
    prev[source] = None
    for stop_id in stop_ids:
        if not stop_id == source:
            arrival_time[stop_id] = INFINITY
            prev[stop_id] = None
        heapq.heappush(Q, (arrival_time[stop_id], stop_id))

    while len(Q) > 0:
        arrive_at_curr, curr = heapq.heappop(Q)

        edges = get_edges_after_time(stop_edge_map[curr], arrive_at_curr)
        for edge in edges:
            neighbor = edge.destination.stop_id
            arrive_at_neighbor = edge.get_arrival_time(arrive_at_curr)
            
            if arrive_at_neighbor < arrival_time[neighbor]:
                arrival_time[neighbor] = arrive_at_neighbor
                prev[neighbor] = curr
                Q = decrease_key(Q, (arrive_at_neighbor, neighbor))

    return arrival_time, prev
        
def path_to_nearest_unvisited_stop(nodes, stop_edge_map, visited, source, time):
    min_unvisited = None
    while not min_unvisited:
        stop_ids = [node.stop_id for node in nodes]
        dist, prev = dijkstra(stop_ids, stop_edge_map, source, time)

        unvisited = [stop_id for stop_id in stop_ids if not stop_id in visited]
        min_unvisited = unvisited[0]
        min_dist = sys.maxint
        for stop in unvisited:
            if dist[stop] < min_dist:
                min_unvisited = stop
                min_dist = dist[stop]

        if unvisited[0] == min_unvisited and prev[min_unvisited] == (None, None):
            print "Could not find a path to stop " + min_unvisited + ".  Removing it."
            nodes = [node for node in nodes if not node.stop_id == min_unvisited]
            min_unvisited = None

    curr = min_unvisited
    path = [(curr, time)]
    while not prev[curr][0] == source:
        curr, time = prev[curr]
        path.insert(0, (curr, time))

    assert prev[path[0][0]][0] == source and path[-1][0] == min_unvisited
        
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
    time = convert_time("00:00:00") # midnight
    visited = [current]
    path = [(current, time)]

    while (len(visited) < len(nodes)):
        time_keyes = sorted(stop_edge_map[current], 
                            key=lambda t: abs(time - t))
        if len(time_keyes) == 0:
            # Ryan, you copied this code from below and you should feel bad
            path_to_nearest = path_to_nearest_unvisited_stop(nodes,
                                 stop_edge_map, visited, 
                                 current, time)
            current, time = path_to_nearest.pop()
            path += path_to_nearest
        else:
            time_key = time_keyes[0]
            current_edges = stop_edge_map[current][time_key]
            if stop_edge_map[current].has_key(ANYTIME):
                current_edges += stop_edge_map[current][ANYTIME]
 
            sorted_edges = sorted(current_edges, key=lambda edge: edge.weight)

            unvisited_edges = [edge for edge in sorted_edges
                if not get_parent_stop_id(edge.destination.stop_id) in visited]

            if (len(unvisited_edges) > 0):
                shortest_edge = unvisited_edges[0]
                current = get_parent_stop_id(shortest_edge.destination.stop_id)
                time = shortest_edge.get_arrival_time(time)
            else:
                path_to_nearest = path_to_nearest_unvisited_stop(nodes,
                                     stop_edge_map, visited, 
                                     current, time)
                current, time = path_to_nearest.pop()
                path += path_to_nearest

        visited.append(current)
        path.append((current, time))

    return path

def benchmark():
    import time
    start_time = time.time()
    s, nodes, stop_edge_map = main()
    nn(s, nodes, stop_edge_map) 
    print("--- %s seconds ---" % (time.time() - start_time))

def get_stop(stops, stop_id):
    stop_id = get_parent_stop_id(stop_id)
    for stop in stops:
        if stop.stop_id == stop_id:
            return stop

    return None

def visualize(nodes, path, old_schedule):
    new_schedule = transitfeed.Schedule()
    path = set(path)
    for stop in path:
        stop = get_stop(nodes, stop)
        stop._schedule = None
        new_schedule.AddStopObject(stop)

    new_schedule.WriteGoogleTransitFeed("nn_path.zip")

    for stop in path:
        stop = get_stop(nodes, stop)
        stop._schedule = old_schedule
    
if __name__ == '__main__':
    s, nodes, stop_edge_map = main()
    path = nn(s, nodes, stop_edge_map)

    visualize(nodes, path, s)
