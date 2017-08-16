#!/usr/bin/env python
#
from dronekit import connect, VehicleMode

# Connect to UDP endpoint.
vehicle = connect('tcp:127.0.0.1:14551', wait_ready=True)
# Use returned Vehicle object to query device state - e.g. to get the mode:
print " Mode: %s" % vehicle.mode.name