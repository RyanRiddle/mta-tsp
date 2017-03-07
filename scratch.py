import transitfeed



print "hello"
feed_filename = './data/google_transit.zip'
schedule = transitfeed.Schedule()
schedule.Load(feed_filename)
print schedule.GetAgencyList().encode('utf-8')
print "ok"
    