import transitfeed
import mta


print "hello"
feed_filename = './data/google_transit.zip'
schedule = transitfeed.Schedule()
schedule.Load(feed_filename)

all_edges = mta.get_edges_with_schedule(schedule)

stops_and_edges = {}

for stop in schedule.stops.values():
    stops_and_edges[stop.stop_id] = mta.get_edges_for_stop(all_edges, stop)


print len(stops_and_edges)
print len(schedule.stops.values())