#!/usr/bin/env python

import time
import picamera
import os
import numpy as np

import signal
import subprocess

from flight_controller import FlightController 
from os import statvfs

# Create an array representing a 1280x720 image of
# a cross through the center of the display. The shape of
# the array must be of the form (height, width, color)
a = np.zeros((576, 720, 3), dtype=np.uint8)
a[360, :, :] = 0x11
a[:, 640, :] = 0x11

is_recording = False
recording_name = ""

camera = picamera.PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 50
#camera.zoom = (0, 0, 1.0, 0.5625)
preview = camera.start_preview()
#preview.fullscreen = True
preview.noaspect = True
#preview.crop = (0, 0, 1280, 720)
preview.window = (0, 0, 720, 576)
camera.annotate_background = picamera.Color('black')
camera.annotate_text = "Connecting..."
#preview.window = (50, 0, 630, 400)
#720x576
#preview.window = (0, 0, 1024, 576)

#while True:
#   time.sleep(2)

#preview.alpha = 255
# Add the overlay directly into layer 3 with transparency;
# we can omit the size parameter of add_overlay as the
# size is the same as the camera's resolution

#o = camera.add_overlay(np.getbuffer(a), (576, 720), layer=3, alpha=64, format='rgb')

flight_controller = FlightController()
flight_controller.connect()

def start_record():
	recording_name = "/data/storage/%d" % int(time.time())
	# The os.setsid() is passed in the argument preexec_fn so
	# it's run after the fork() and before  exec() to run the shell.
	proc = subprocess.Popen("arecord -D plughw:1,0 -v -f cd -t raw | lame -r -b 192 - %s.mp3" % recording_name, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid) 
	camera.start_recording(recording_name + ".h264", quality=15)
	
def stop_record():
	camera.stop_recording()
	os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
	#os.system("MP4Box -fps 50 -tmp /data/storage/tmp -add %s %s && rm %s" % (recording_name + ".h264", recording_name + ".mp4", recording_name + ".h264"))
	#os.system("MP4Box -fps 50 -tmp /data/storage/tmp -add %s %s && rm %s" % (recording_name + ".mp3", recording_name + ".mp4", recording_name + ".mp3"))
	# MP4Box -add filename.h264 filename.mp4
	is_recording = False

try:
    # Wait indefinitely until the user terminates the script
    disk_space_i = 0
    disk_space = 0
    proc = 0
    while True:
        time.sleep(.25)
        b = flight_controller.battery
        t = (b.percentage, b.voltage, b.current, flight_controller.groundspeed, flight_controller.altitude)
        text = " %0.0f%% (%0.1f V) %0.1f A, %0.1f km/h, %0.0f m " % t

        #print flight_controller.channels

        if flight_controller.channels[7] > 1800 and not is_recording:
					start_record()
        elif flight_controller.channels[7] < 1800 and is_recording:
          stop_record()

        if is_recording:
          if disk_space_i == 0:
             s = statvfs("/data/storage")
             disk_space = (s.f_bavail * s.f_frsize) / (1024.0 * 1024.0 * 1024.0) # in GB
          text = (" [Recording, Free: %0.2fGB] \n" % disk_space) + text
          disk_space_i = (disk_space_i + 1) % 16

        if flight_controller.is_armed:
          camera.annotate_text = text
          if not is_recording:
          	start_record()
        else:
          camera.annotate_text = " Unarmed \n" + text
          if is_recording:
          	stop_record()

        # a = a + 0x11
        # try:
        #   o.update(np.getbuffer(a))
        # except Exception, err:
        #   print 'error'
        # print 'update'
finally:
  print "done"
  #camera.remove_overlay(o)
