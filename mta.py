import sys
import transitfeed

DATA_FILENAME = './data/google_transit.zip'
SERVICE_PERIODS = [
    u"A20161106WKD",
    u"B20161106WKD",
    u"R20161106WKD"
]
ANYTIME = 999999

def convert_time(time_string):
    time_string = time_string.replace(':', '')
    return int(time_string)

class Edge(object):
    def __init__(self, origin, destination, weight=None):
        self.origin = origin
        self.destination = destination
        if weight is None:
            self.weight = convert_time(destination.arrival_time) - convert_time(origin.departure_time)
            self.is_transfer = False
        else:
            self.weight = weight
            self.is_transfer = True

    def get_departure_time(self, current_time):
        if self.is_transfer:
            return current_time
        else:
            return convert_time(self.origin.departure_time)

    def get_arrival_time(self, current_time):
        if self.is_transfer:
            return current_time + self.weight
        else:
            return convert_time(self.destination.arrival_time)

def seconds_to_weird_time(seconds):
    seconds_to_time = {}
    seconds_to_time[0] = 0
    seconds_to_time[90] = 130
    seconds_to_time[120] = 200
    seconds_to_time[180] = 300
    seconds_to_time[300] = 600

    assert seconds in seconds_to_time

    return seconds_to_time[seconds]

def get_transfers_from_schedule(schedule):
    transfers_ll = schedule._transfers.values()
    return [transfer for transfer_l in transfers_ll for transfer in transfer_l]

def build_edges_from_transfers(transfers, nodes):
    return [ Edge(get_stop(nodes, transfer.from_stop_id),
                  get_stop(nodes, transfer.to_stop_id),
                  seconds_to_weird_time(transfer.min_transfer_time))
            for transfer in transfers]
    

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

def get_edges_with_schedule(schedule, nodes):
    trips = [t for t in schedule.trips.values() if t.service_id in SERVICE_PERIODS]
    edges_per_trip = [build_edges_from_trip(trip) for trip in trips]

    trip_edges = [e for edges in edges_per_trip for e in edges]

    transfer_edges = build_edges_from_transfers(get_transfers_from_schedule(schedule), nodes)

    return trip_edges + transfer_edges

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

    time_map = {}
    for edge in es:
        if not edge.is_transfer:
            time = edge.get_departure_time(None)
        else:
            time = ANYTIME
        if time_map.has_key(time):
            time_map[time] += [edge]
        else:
            time_map[time] = [edge]

    return time_map

def get_parent_stop_id(stop_id):
    if stop_id[-1] in ('N', 'S'):
        return stop_id[0:-1]
    else:
        return stop_id

def getSchedule():
    schedule = transitfeed.Schedule()
    schedule.Load(DATA_FILENAME)
    return schedule

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

def build_edge_index(stops, edges):
    stops_and_edges = {}

    for stop in stops:
        parent_stop = get_parent_stop_id(stop.stop_id)
        if stops_and_edges.has_key(parent_stop):
            stops_and_edges[parent_stop] = dict_concat(stops_and_edges[parent_stop], get_edges_for_stop(edges, stop))
        else:
            stops_and_edges[parent_stop] = get_edges_for_stop(edges, stop)

    print len(stops_and_edges)
    return stops_and_edges

def isChildStop(stop):
    return stop.stop_id[-1] in (u'N', u'S')

def main():
    s = getSchedule()

    stops = s.stops.values()
    edges = get_edges_with_schedule(s, stops)
    stop_edge_map = build_edge_index(stops, edges)
    nodes = [stop for stop in stops if not isChildStop(stop) ]
    nodes = prune_stops(nodes, stop_edge_map)
    nodes = remove_blacklisted(nodes)

    return s, nodes, stop_edge_map

def dijkstra(stop_ids, stop_edge_map, source, time):
    dist = {}
    prev = {}
    Q = []

    dist[source] = 0
    prev[source] = (None, None)
    for stop_id in stop_ids:
        if not stop_id == source:
            dist[stop_id] = sys.maxint     # infinity
            prev[stop_id] = (None, None)
        Q.append((stop_id, time, dist[stop_id]))

    sorted_Q = Q        #will sort later
    while len(sorted_Q) > 0:
        sorted_Q = sorted(sorted_Q, key=lambda x: x[2])
        tup = sorted_Q.pop(0)
        curr = tup[0]
        time = tup[1]

        for edges in stop_edge_map[curr].values():
            for edge in edges:
                neighbor = get_parent_stop_id(edge.destination.stop_id)
                t = edge.get_arrival_time(time)
                alt = dist[curr] + edge.weight + abs(time - t)
                if alt < dist[neighbor]:
                    dist[neighbor] = alt
                    prev[neighbor] = (curr, time)

                    i = [i for i, tup in enumerate(sorted_Q) if tup[0] == neighbor][0]
                    sorted_Q[i] = (neighbor, t, alt)

    return dist, prev
        
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
