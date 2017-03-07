import itertools

class Edge(object):
    def __init__(self, origin, destination):
        self.origin = origin
        self.destination = destination

def get_edges_with_trip(trip):
    stop_times = trip.GetStopTimes()

    edges = []
    # stop_times seems to be sorted by sequence_num already
    for i in range(len(stop_times) - 1):
        time = stop_times[i]
        next_time = stop_times[i+1]

        edge = Edge(time.stop_id, next_time.stop_id)
        edges.append(edge)

    return edges

def get_trips_with_schedule(schedule):
    services = [u"A20161106WKD", u"B20161106WKD", u"R20161106WKD"]
    trips = schedule.trips.values()
    return [trip for trip in trips if trip.service_id in services]

def get_edges_with_schedule(schedule):
    trips = get_trips_with_schedule(schedule)
    aa = [get_edges_with_trip(trip) for trip in trips]

    edges = [edge for a in aa for edge in a]
    return edges
