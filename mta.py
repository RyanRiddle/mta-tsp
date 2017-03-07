import itertools
import transitfeed

DATA_FILENAME = './data/google_transit.zip'
SERVICE_PERIODS = [
    u"A20161106WKD",
    u"B20161106WKD",
    u"R20161106WKD"
]

class Edge(object):
    def __init__(self, origin, destination):
        self.origin = origin
        self.destination = destination
        self.weight = this.destination.arrival_time - this.origin.departure_time

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
    stop.stop_id[-1] in ('N', 'S')

def main():
    s = getSchedule()

    stops = s.stops.values()
    edges = get_edges_with_schedule(s)
    stop_edge_map = build_edge_index(stops, edges)
    nodes = [stop for stop in stops if not isChildStop(stop) ]

    current = nodes[0]
    visited = [current.stop_id]
    visited_edges = []
    unique_visited = [current.stop_id]

    while (len(unique_visited) < len(nodes)):
        es = sorted(stop_edge_map[current.stop_id], key=Edge.weight)
        current_edges = es.remove(visited_edges[-1])

        unvisited_edges = [edge for edge in current_edges if not edge.destination.stop_id in visited]

        if (len(unvisited_edges) > 0):
            shortest_edge = unvisited_edges[0]
            visited.append(shortest_edge.destination.stop_id)
            unique_visited.append(shortest_edge.destination.stop_id)
            visited_edges.append(shortest_edge)
        else:
            # find shortest edge back
            shortest_edge = current_edges[0]
            visited.append(shortest_edge.destination.stop_id)
            visited_edges.append(shortest_edge)

    print visited


if __name__ == '__main__':
    import time
    start_time = time.time()
    main()
    print("--- %s seconds ---" % (time.time() - start_time))
