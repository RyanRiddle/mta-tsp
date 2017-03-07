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
    return [edge for edge in all_edges if edge.origin.stop_id == stop.stop_id]


def build_edges_for_stop(stop, schedule):
    trips = [t for t in schedule.trips.values() if t.service_id in SERVICE_PERIODS]
    for t in trips:
        stop_times = t.GetStopTimes()
        edges = []
    # stop_times seems to be sorted by sequence_num already
    for i in range(len(stop_times) - 1):
        if stop_times[i].stop_id == stop.stop_id:
            next_time = stop_times[i+1]
            edge = Edge(stop_times[i], next_time)
            edges.append(edge)

    return edges

def getSchedule():
    schedule = transitfeed.Schedule()
    schedule.Load(DATA_FILENAME)
    return schedule

def main():
    s = getSchedule()

    stops = s.stops.values()
    edges = get_edges_with_schedule(s)

    stops_and_edges = {}

    for stop in stops:
        build_edges_for_stop(stop, schedule)
        stops_and_edges[stop.stop_id] = build_edges_for_stop(stop, schedule)
    print len(stops_and_edges)


if __name__ == '__main__':
    import time
    start_time = time.time()
    main()
    print("--- %s seconds ---" % (time.time() - start_time))