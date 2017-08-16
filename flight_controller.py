import thread

from dotmap import DotMap
from pymavlink import mavutil

class FlightController:
  """docstring for FlightController"""
  def __init__(self):
    #super(FlightController, self).__init__()
    #self.arg = arg
    self.controller = None

    self.channels = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    self.is_armed = False
    self.is_enabled = False
    self.battery = DotMap(voltage=0, current=0, percentage=0)
    self.groundspeed = 0
    self.altitude = 0
  
  def connect(self):
    # create a mavlink serial instance
    self.controller = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

    # wait for the heartbeat msg to find the system ID
    self.controller.wait_heartbeat()

    # request data to be sent at the given rate
    # This is actually deprecated, switch to using MAV_CMD_SET_MESSAGE_INTERVAL
    self.rate = 4 # 4hz
    self.controller.mav.request_data_stream_send(self.controller.target_system, self.controller.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, self.rate, 1)
    self.controller.wait_heartbeat()

    # enter the data loop
    thread.start_new_thread(self.read_loop, ())

  def handle_rc_raw(self, msg): 
    # RC_CHANNELS_RAW {time_boot_ms : 58004, port : 0, chan1_raw : 1499, chan2_raw : 1499, chan3_raw : 1016, chan4_raw : 1499, chan5_raw : 1065, chan6_raw : 1499, chan7_raw : 1000, chan8_raw : 1499, rssi : 0}
    self.channels = [0, msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw] # 1-indexed array

  def handle_heartbeat(self, msg):
    mode = mavutil.mode_string_v10(msg)
    old_armed = self.is_armed
    self.is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    self.is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

    if not old_armed and self.is_armed:
      print "requesting stream"
      self.controller.mav.request_data_stream_send(self.controller.target_system, self.controller.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, self.rate, 1)

  def handle_hud(self, msg):
    # VFR_HUD {airspeed : 0.0, groundspeed : 0.0, heading : 230, throttle : 0, alt : 0.0, climb : 0.019999999553}
    self.groundspeed = msg.groundspeed
    self.altitude = msg.alt

  def handle_sys_status(self, msg):
    # SYS_STATUS {onboard_control_sensors_present : 2161711, onboard_control_sensors_enabled : 2137135, onboard_control_sensors_health : 2161711, load : 610, voltage_battery : 798, current_battery : 437, battery_remaining : 99, drop_rate_comm : 0, errors_comm : 0, errors_count1 : 0, errors_count2 : 0, errors_count3 : 0, errors_count4 : 0}
    self.battery.voltage = msg.voltage_battery / 1000.0
    self.battery.current = msg.current_battery / 100.0
    self.battery.percentage = msg.battery_remaining

  def read_loop(self):
    while(True):
      # grab a mavlink message
      msg = self.controller.recv_match(blocking=True)
      #print msg
      if not msg:
        continue
        #return

      # handle the message based on its type
      msg_type = msg.get_type()
      if msg_type == "BAD_DATA":
        continue
        #if mavutil.all_printable(msg.data):
          #print "Bad Data"
          #print msg
          #sys.stdout.write(msg.data)
          #sys.stdout.flush()
      elif msg_type == "RC_CHANNELS_RAW": 
        self.handle_rc_raw(msg)
      elif msg_type == "HEARTBEAT":
        self.handle_heartbeat(msg)
      elif msg_type == "VFR_HUD":
        self.handle_hud(msg)
      #elif msg_type == "ATTITUDE":
      #  handle_attitude(msg)
      elif msg_type == "SYS_STATUS":
        self.handle_sys_status(msg)
