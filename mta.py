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
INFINITY = sys.maxint
TWENTY_FOUR_HOURS = 24 * 60 * 60

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

def build_edges_from_transfers(transfers, node_map):
    edges = []
    for transfer in transfers:
        origin      = node_map[transfer.from_stop_id]
        destination = node_map[transfer.to_stop_id]
        edge = Edge(origin, destination, ANYTIME, transfer.min_transfer_time)
        edges.append(edge)
    
    return edges

def build_edges_from_trip(trip, node_map):
    stop_times = trip.GetStopTimes()

    edges = []
    # stop_times is sorted by stop_sequence
    for i in range(len(stop_times) - 1):
        stop_time = stop_times[i]
        origin    = node_map[get_parent_stop_id(stop_time.stop_id)]
        depart_at = seconds_past_midnight(stop_time.departure_time)

        next_stop_time  = stop_times[i+1]
        destination     = node_map[get_parent_stop_id(next_stop_time.stop_id)]
        arrive_at       = seconds_past_midnight(next_stop_time.arrival_time)

        edge = Edge(origin, destination, depart_at, arrive_at)
        edges.append(edge)

    return edges

def get_edges_with_schedule(schedule, node_map):
    trips = [trip for trip in schedule.trips.values() 
                if trip.service_id in SERVICE_PERIODS]

    edges_per_trip = [build_edges_from_trip(trip, node_map) for trip in trips]

    trip_edges = [edge for edges in edges_per_trip 
                       for edge in edges]

    transfers = get_transfers_from_schedule(schedule) 
    transfer_edges = build_edges_from_transfers(transfers, node_map)

    return trip_edges + transfer_edges

def get_parent_stop_id(stop_id):
    if stop_id[-1] in ('N', 'S'):
        return stop_id[0:-1]
    else:
        return stop_id

def prune_stops(node_map, edges):
    return {stop_id: stop for stop_id, stop in node_map.iteritems()
                          if edges.has_key(stop_id) and 
                                len(edges[stop_id]) > 0}

def remove_blacklisted(node_map):
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
    return {k: v for k, v in node_map.iteritems() if k not in blacklist}

def get_edges_for_stop(all_edges, stop_id):
    return [edge for edge in all_edges if edge.origin.stop_id == stop_id]

def build_edge_index(node_map, all_edges):
    stops_and_edges = {}

    for stop_id in node_map:
        edges   = get_edges_for_stop(all_edges, stop_id)
        sorted_edges = sorted(edges, key=lambda edge: edge.depart_at)
        stops_and_edges[stop_id] = sorted_edges

    return stops_and_edges

def is_child_stop(stop_id):
    return stop_id[-1] in ('N', 'S')

def load_schedule():
    schedule = transitfeed.Schedule()
    schedule.Load(DATA_FILENAME)
    return schedule

def get_nodes(schedule):
    return {k: v for k, v in schedule.stops.iteritems()
                    if not is_child_stop(k)}

def main():
    schedule = load_schedule()
    node_map = get_nodes(schedule)

    edges = get_edges_with_schedule(schedule, node_map)
    stop_edge_map = build_edge_index(node_map, edges)

    node_map = prune_stops(node_map, stop_edge_map)
    node_map = remove_blacklisted(node_map)

    return schedule, node_map, stop_edge_map

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
        
def _dfs(stop_id, stop_edge_map, visited):
    visited.append(stop_id)
    current_edges = sorted(stop_edge_map[stop_id], key=lambda edge: edge.weight)

    for neighbor in current_edges:
        neighbor_id = get_parent_stop_id(neighbor.destination.stop_id)
        if not neighbor_id in visited:
            visited = _dfs(neighbor_id, stop_edge_map, visited)

    return visited
    
def dfs(node_map, stop_edge_map):
    visited = []

    while len(visited) < len(node_map):
        stop_id = [stop_id for stop_id in node_map if not node.stop_id in visited][0]
        visited = _dfs(stop_id, stop_edge_map, visited) 

    return visited

def path_to_nearest_unvisited_stop(node_map, stop_edge_map, visited, source, time):
    stop_ids = [stop_id for stop_id in node_map]

    
    arrival_times, previous_stops = dijkstra(stop_ids, stop_edge_map,
                                             source, time)

    unvisited = [stop_id for stop_id in stop_ids if not stop_id in visited]
    min_unvisited = None
    min_arrival_time = INFINITY
    for stop in unvisited:
        if arrival_times[stop] < min_arrival_time:
            min_unvisited = stop
            min_arrival_time = arrival_times[stop]

    if min_unvisited is None and time > TWENTY_FOUR_HOURS:
        time -= TWENTY_FOUR_HOURS
        path =  path_to_nearest_unvisited_stop(stop_ids, stop_edge_map,
                                              visited, source, time)
        return [(stop_id, arrival_time + TWENTY_FOUR_HOURS)
                for stop_id, arrival_time in path]

    curr = min_unvisited
    arrival_time = arrival_times[curr]
    path = [(curr, arrival_time)]

    while not previous_stops[curr] == source:
        curr = previous_stops[curr]
        arrival_time = arrival_times[curr]
        path.insert(0, (curr, arrival_time))
        
    return path

def nearest_neighbor(schedule, node_map, stop_edge_map):
    time = seconds_past_midnight("00:00:00")
    current = node_map.keys()[-1]
    visited = [current]
    path = [(current, time)]

    while (len(visited) < len(node_map)):
        current_edges = get_edges_after_time(stop_edge_map[current], time)

        sorted_edges = sorted(current_edges, 
                              key=lambda edge: edge.get_arrival_time(time))

        unvisited_edges = [edge for edge in sorted_edges
                                if not edge.destination.stop_id in visited]

        if (len(unvisited_edges) > 0):
            shortest_edge = unvisited_edges[0]
            current = shortest_edge.destination.stop_id
            time = shortest_edge.get_arrival_time(time)
        else:
            path_to_nearest = path_to_nearest_unvisited_stop(node_map,
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
    s, node_map, stop_edge_map = main()
    nn(s, node_map, stop_edge_map) 
    print("--- %s seconds ---" % (time.time() - start_time))

def visualize(node_map, path, old_schedule):
    new_schedule = transitfeed.Schedule()
    path = set(path)
    for stop in path:
        stop = node_map[stop]
        stop._schedule = None
        new_schedule.AddStopObject(stop)

    new_schedule.WriteGoogleTransitFeed("nn_path.zip")

    for stop in path:
        stop = node_map[stop]
        stop._schedule = old_schedule
    
if __name__ == '__main__':
    s, node_map, stop_edge_map = main()
    path = nn(s, node_map, stop_edge_map)

    visualize(node_map, path, s)
