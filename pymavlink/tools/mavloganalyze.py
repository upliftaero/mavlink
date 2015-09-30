#!/usr/bin/env python

'''
Breakout MAVLink logs by flights etc.
Based on mavsummarize.py.
'''

import sys, time, os, errno, glob, pprint, math
import log_meta_keys as keys
#import ../mavparam

# TODO: Looks like the first flight maximums are wrong

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("logs", metavar="LOG", nargs="+", help="Log files to convert")

# TODO: Do we use all of these arguments?

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.mavextra import distance_two

TAKEOFF_AIRSPEED = 4.0      # meters / second
TAKEOFF_LAND_DETECTION_HYSTERESIS = 5.0     # seconds

def TimestampString(timestamp):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))

def TwoDec(number):
    return "{:.2f}".format(number)

def MMSSTime(time):
    return "{:3.0f}:{:02.0f}".format(int(time / 60), time % 60)

def create_path_if_needed(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

# Represents the statistics of a single data parameters
#   See: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
#   Extend this to weighted statistics??
class DataStatistics():         # Should be a dict?

    def __init__(self):     # Have a name?
        self.n      = 0
        self.sum    = 0.0
        self.M2     = 0.0
        self.min    = None
        self.max    = None
        self.mean   = 0.0

    def accumulate(self, value):
        self.n += 1
        self.sum += value
        if self.min is None or value < self.min:
            self.min = value
        if max is None or value > self.max:
            self.max = value
        delta = value - self.mean
        self.mean = self.mean + delta/self.n
        self.M2 = self.M2 + delta*(value - self.mean)


    def get_stats(self):
        stats = {}
        if self.n > 1:
            variance = self.M2 / (self.n - 1)
            stats[keys.STATS_VARIANCE]   = variance
            stats[keys.STATS_STDEV]      = math.sqrt(variance)
        stats[keys.STATS_MIN]        = self.min
        stats[keys.STATS_MAX]        = self.max
        stats[keys.STATS_MEAN]       = self.mean
        if self.n != 0:
            stats["other_mean"] = self.sum / self.n
        stats[keys.STATS_COUNT]      = self.n
        return stats


class Flight(dict):

    def __init__(self, logfile, flight_number):
        dict.__init__(self)
        #self['file_name'] = logfile
        self[keys.FLIGHT_FLIGHT_NUMBER] = flight_number
        self.takeoff_count = 0
        self.landing_count = 0
        self.initial_mode = ""
        self.mode_changes = []
        self.flight_time = 0.0
        self.flying = False
        self.last_takeoff_time = 0.0
        self.last_landing_time = 0.0

        self.altitude_stats = DataStatistics();
        self.airspeed_stats = DataStatistics();
        self.throttle_stats = DataStatistics();
        self.elevator_stats = DataStatistics();
        self.aileron_stats = DataStatistics();


    def Takeoff(self, time):      # May not have been an actual detected landing - add a reason code??
        self.flying = True
        self.takeoff_count += 1
        self.last_takeoff_time = time

    def Land(self, time):      # May not have been an actual detected landing - add a reason code??
        self.last_landing_time = time
        self.flight_time += (self.last_landing_time - self.last_takeoff_time)
        self.flying = False
        self.landing_count += 1

    def EndArmed(self, total_time):
        if self.landing_count != 1 or self.takeoff_count != 1:
            print("Unexpected takeoff/landing count: " + str(self.takeoff_count) + " / " + str(self.landing_count))
        self[keys.FLIGHT_TAKEOFF_COUNT]     = self.takeoff_count
        self[keys.FLIGHT_LANDING_COUNT]     = self.landing_count
        self[keys.FLIGHT_FLIGHT_TIME]       = self.flight_time
        self[keys.FLIGHT_FLIGHT_TIME_STR]   = MMSSTime(self.flight_time)
        self[keys.FLIGHT_START_TIME]        = self.last_takeoff_time
        self[keys.FLIGHT_START_TIME_STR]    = TimestampString(self.last_takeoff_time)
        self[keys.FLIGHT_END_TIME]          = self.last_landing_time
        self[keys.FLIGHT_END_TIME_STR]      = TimestampString(self.last_landing_time)
        self[keys.FLIGHT_TOTAL_TIME]        = total_time
        self[keys.FLIGHT_MODE_CHANGES]      = self.mode_changes
        self[keys.FLIGHT_INITIAL_MODE]      = self.initial_mode
        self[keys.FLIGHT_ALTITUDE_STATS]    = self.altitude_stats.get_stats()
        self[keys.FLIGHT_AIRSPEED_STATS]    = self.airspeed_stats.get_stats()
        self[keys.FLIGHT_THROTTLE_STATS]    = self.throttle_stats.get_stats()
        self[keys.FLIGHT_ELEVATOR_STATS]    = self.elevator_stats.get_stats()
        self[keys.FLIGHT_AILERON_STATS]     = self.aileron_stats.get_stats()

class LogFile(dict):

    # Variables declared here are CLASS variables

    def __init__(self, filename, aircraft_type):
        dict.__init__(self)

        # This is how instance variables are declared
        # Not all of these need to be intance variables
        self.flights = []

        self.file_name = filename

        self.autonomous_sections = 0     # How many different autonomous sections there are
        self.auto_time = 0.0             # The total time the vehicle was autonomous/guided (seconds)

        self.armed_sections = 0          # How many different armed sections there are
        self.armed_time = 0.0            # The total time the vehicle was armed (seconds)

        self.start_time = None           # The datetime of the first received message (seconds since epoch)
        self.total_dist = 0.0            # The total ground distance travelled (meters)
        self.true_time = None            # Track the first timestamp found that corresponds to a UNIX timestamp

        # Do we need to weight the cum's / counts by time delta?
        self.hud_records = []
        self.hud_count = 0

        self.altitude_deltas = []
        self.altitude_last = 0
        self.altitude_window = 0

        self.total_time = 0
        self.record_count = 0;

        self.flight_number = 1

        self.last_gps_msg    = None          # The last GPS message received
        self.first_gps_msg   = None          # The first GPS message received
        self.aircraft_type   = aircraft_type

        # Open the log file
        self.mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps, dialect=args.dialect)

    def AnalyzeLog(self):

        autonomous      = False         # Whether the vehicle is currently autonomous at this point in the logfile
        armed           = False         # Whether the vehicle is currently armed at this point in the logfile
        current_mode    = None          # Track mode changes
        climb_state     = 0             # For now, 0 = level, 1 = climb, 2 = descent -- need to make this an enum
        vsi_state       = 0
        flight          = None          # These two are redundant
        start_auto_time = 0
        airspeed_rise_start = 0
        airspeed_drop_start = 0
        parameters      = {}  # Ultimately: MAVParmDict()

        while True:
            m = self.mlog.recv_match(condition=args.condition)

            # If there's no match, it means we're done processing the log.
            if m is None:
                break
            self.record_count += 1;

            # Ignore any failed messages
            if m.get_type() == 'BAD_DATA':
                continue

            # Keep track of the latest timestamp for various calculations
            timestamp = getattr(m, '_timestamp', 0.0)

            # Log the first message time
            if self.start_time is None:
                self.start_time = timestamp

            # Log the first timestamp found that is a true timestamp. We first try
            # to get the groundstation timestamp from the log directly. If that fails
            # we try to find a message that outputs a Unix time-since-epoch.
            if self.true_time is None:
                if not args.notimestamps and timestamp >= 1230768000:
                    self.true_time = timestamp
                elif 'time_unix_usec' in m.__dict__ and m.time_unix_usec >= 1230768000:
                    self.true_time = m.time_unix_usec * 1.0e-6
                elif 'time_usec' in m.__dict__ and m.time_usec >= 1230768000:
                    self.true_time = m.time_usec * 1.0e-6

            # Track the vehicle's speed and status
            if m.get_type() == 'GPS_RAW_INT':

                # Ignore GPS messages without a proper fix
                if m.fix_type < 3 or m.lat == 0 or m.lon == 0:
                    continue

                # Log the first GPS location found, being sure to skip GPS fixes
                # that are bad (at lat/lon of 0,0)
                if self.first_gps_msg is None:
                    self.first_gps_msg = m

                # Track the distance travelled, being sure to skip GPS fixes
                # that are bad (at lat/lon of 0,0)
                if self.last_gps_msg is not None:
                    self.total_dist += distance_two(self.last_gps_msg, m)

                # Save this GPS message to do simple distance calculations with
                self.last_gps_msg = m

            elif m.get_type() == 'PARAM_VALUE':
                pname = m.param_id
                if pname in parameters and parameters[pname] != m.param_value:
                    print("Parameter changed in file: " + pname + ": " + str(parameters[pname]) + " to " + str(m.param_value))
                parameters[pname] = m.param_value

            elif m.get_type() == 'HEARTBEAT' and m.type != mavutil.mavlink.MAV_TYPE_GCS:
                # Track autonomous time
                if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED or
                        m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and autonomous == False:
                    autonomous = True
                    self.autonomous_sections += 1
                    start_auto_time = timestamp
                elif (not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED and
                    not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and autonomous == True:
                    autonomous = False
                    self.auto_time += timestamp - start_auto_time

                # Track armed time
                if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and armed == False:
                    armed = True
                    self.armed_sections += 1

                    # This is the logic to start a flight
                    start_armed_time = timestamp
                    print("==== Flight #" + str(self.flight_number) + "====")
                    flight = Flight(self.file_name, self.flight_number)
                    flight.initial_mode = self.mlog.flightmode
                    self.flight_number += 1
                    self.flights.append(flight)
                elif (not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and armed == True:
                    armed = False
                    self.armed_time += timestamp - start_armed_time
                    if flight.flying:
                        print("Landing (by end of arm)!! at: " + TimestampString(timestamp))
                        flight.Land(timestamp - airspeed_rise_start)
                    flight.EndArmed(timestamp - start_armed_time)
                    flight = None

                if current_mode != m.base_mode:
                    if (m.base_mode != 0):   # Quick hack to make it work.  Need to understand HEARTBEATS in detail
                        mode_string = ""
                        if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                            mode_string += "ARMED "
                        if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED):
                            mode_string += "GUIDED "
                        if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED):      # Note - this is never turned on in our logs??
                            mode_string += "AUTO "
                        #print(TimestampString(timestamp) + ": Mode changed to 0x" + format(m.base_mode, '02x') + " " +
                        #            self.mlog.flightmode + " / " + mode_string)     # Need to convert to local time here??
                        if flight is not None and flight.flying:
                            mode_change = {}
                            # Need to use the more decoded modes
                            mode_change[keys.MODE_CHANGE_TIME] = timestamp
                            mode_change[keys.MODE_CHANGE_OLD_MODE] = current_mode
                            mode_change[keys.MODE_CHANGE_NEW_MODE] = m.base_mode
                            mode_change[keys.MODE_CHANGE_MODE_NAME] = self.mlog.flightmode
                            flight.mode_changes.append(mode_change)
                        current_mode = m.base_mode


            elif m.get_type() == 'SERVO_OUTPUT_RAW' and armed and flight.flying:
                flight.elevator_stats.accumulate(m.servo2_raw)
                flight.aileron_stats.accumulate(m.servo4_raw)


            elif m.get_type() == 'VFR_HUD' and armed:
                if flight.flying:
                    # Accumulate flight statistics
                    flight.altitude_stats.accumulate(m.alt)
                    flight.airspeed_stats.accumulate(m.airspeed)
                    flight.throttle_stats.accumulate(m.throttle)

                self.hud_records.append(m)

                # Vertical speed detection logic
                delta = (m.alt - self.altitude_last) if self.hud_count != 0 else 0
                self.altitude_deltas.append(delta)
                self.altitude_window += delta           # Window should really be time based!!
                self.hud_count += 1
                altitude_last = m.alt
                if (self.hud_count >= 5):       # Make all of these things parameters
                    self.altitude_window -= self.altitude_deltas[self.hud_count-4]      # Need to check these indices and boundary conditions rigorously...
                    #print(TimestampString(timestamp) + ": " + "Altitudes: " + TwoDec(m.alt) + " " + TwoDec(altitude_window) + " " + TwoDec(m.climb))
                    new_state = climb_state     # By default, nothing is happening
                    if climb_state == 0:
                        if self.altitude_window > 4.0:
                            new_state = 1
                        elif self.altitude_window < -4.0:
                            new_state = -1
                    elif climb_state == 1:
                        if self.altitude_window < 2.0:       # Use different thresholds to create hysterisis
                            new_state = 0
                    elif climb_state == -1:
                        if self.altitude_window > -2.0:
                            new_state = 0

                    if new_state != climb_state:
                        #print(TimestampString(timestamp) + ": " + ("Levelout" if new_state == 0 else "Climb" if new_state == 1 else "Descent") + " started " + TwoDec(m.alt) + " " + TwoDec(altitude_window))
                        climb_state = new_state

                # Try a simpler method using vertical speed and compare
                #climb_last = m.climb
                new_vsi_state = vsi_state     # By default, nothing is happening
                if vsi_state == 0:
                    if m.climb > 3.0:
                        new_vsi_state = 1
                    elif m.climb < -3.0:
                        new_vsi_state = -1
                elif vsi_state == 1:
                    if m.climb < 1.5:       # Use different thresholds to create hysterisis
                        new_vsi_state = 0
                elif vsi_state == -1:
                    if m.climb > -1.5:
                        new_vsi_state = 0

                if new_vsi_state != vsi_state:
                    #print(TimestampString(timestamp) + ": VSI " + ("Levelout" if new_vsi_state == 0 else "Climb" if new_vsi_state == 1 else "Descent") + " started " + TwoDec(m.alt) + " " + TwoDec(m.climb))
                    vsi_state = new_vsi_state

                if not flight.flying:
                    if m.airspeed > TAKEOFF_AIRSPEED:
                        if airspeed_rise_start == 0:
                            airspeed_rise_start = timestamp
                        elif (timestamp - airspeed_rise_start) > TAKEOFF_LAND_DETECTION_HYSTERESIS:
                            print("Takeoff!! at: " + TimestampString(airspeed_rise_start))
                            flight.Takeoff(airspeed_rise_start)
                    else:
                        airspeed_rise_start = 0
                else:
                    if m.airspeed < TAKEOFF_AIRSPEED:
                        if airspeed_drop_start == 0:
                            airspeed_drop_start = timestamp
                        elif (timestamp - airspeed_drop_start) > TAKEOFF_LAND_DETECTION_HYSTERESIS:
                            print("Landing!! at: " + TimestampString(timestamp))
                            flight.Land(timestamp)
                    else:
                        airspeed_drop_start = 0



        # If there were no messages processed, say so
        if self.start_time is None:
            print("ERROR: No messages found.")
            return

        # If the vehicle ends in autonomous mode, make sure we log the total time
        if autonomous:
            self.auto_time += timestamp - start_auto_time

        # If the vehicle ends in armed mode, make sure we log the total time
        # Combined with desarm detection above
        if armed:
            self.armed_time += timestamp - start_armed_time
            if flight.flying:
                print("Log ended while flying!!!!")
                flight.Land(timestamp)
            flight.EndArmed(timestamp - start_armed_time)
            flight = None


        # Compute the total logtime, checking that timestamps are 2009 or newer for validity
        # (MAVLink didn't exist until 2009)
        if self.true_time:
            print("Log started at about " + TimestampString(self.true_time))
        else:
            print("Warning: No absolute timestamp found in datastream. No starting time can be provided for this log.")

        self.total_time = timestamp - self.start_time
        self[keys.LOG_PARAMETERS] = parameters

        self[keys.LOG_RECORD_COUNT]    = self.record_count
        self[keys.LOG_TOTAL_TIME]      = self.total_time
        self[keys.LOG_AUTONOMOUS_SECTIONS]      = self.autonomous_sections
        self[keys.LOG_AUTONOMOUS_TIME] = self.auto_time
        self[keys.LOG_ARMED_SECTIONS]  = self.armed_sections
        self[keys.LOG_ARMED_TIME]      = self.armed_time
        if "MIXING_GAIN" in self[keys.LOG_PARAMETERS]:
            self[keys.LOG_MIXING_GAIN]     = self[keys.LOG_PARAMETERS]["MIXING_GAIN"]
        self[keys.LOG_TOTAL_DISTANCE]  = self.total_dist
        self[keys.LOG_FLIGHTS]         = self.flights
        self[keys.LOG_FILENAME]        = self.file_name
        self[keys.LOG_AIRCRAFT_TYPE]   = self.aircraft_type


    def PrintSummary(self):
        print
        print("===========" + self.file_name + "===========")
        print("Aircraft type:         " + self.aircraft_type)
        print("Total record count:    " + str(self.record_count))
        print("Total time (mm:ss):    " + MMSSTime(self.total_time))
        # The autonomous time should be good, as a steady HEARTBEAT is required for MAVLink operation
        print("Autonomous sections:   {}".format(self.autonomous_sections))
        if self.autonomous_sections > 0:
            print("Autonomous time (mm:ss): " + MMSSTime(self.auto_time))

        print("Armed sections:        {}".format(self.armed_sections))
        if self.armed_sections > 0:
            print("Armed time (mm:ss):    " + MMSSTime(self.armed_time))

        if "MIXING_GAIN" in self["parameters"]:
            print("Mixing gain:           " + str(self["parameters"]["MIXING_GAIN"]))
        else:
            print("No MIXING_GAIN")
        # Print location data
        if self.last_gps_msg is not None:
            first_gps_position = (self.first_gps_msg.lat / 1e7, self.first_gps_msg.lon / 1e7)
            last_gps_position = (self.last_gps_msg.lat / 1e7, self.last_gps_msg.lon / 1e7)
            print("Travelled from ({0[0]}, {0[1]}) to ({1[0]}, {1[1]})".format(first_gps_position, last_gps_position))
            print("Total distance:        {:0.2f}m".format(self.total_dist))
        else:
            print("Warning: No GPS data found, can't give position summary.")

        print("Flights: ")
        pprint.pprint(self.flights)
        print

# End of LogFile class

logs = {}
param_file_dir = "parameter-files/"

create_path_if_needed(param_file_dir)
for filename in args.logs:
    for f in glob.glob(filename):
        lower_filename = f.lower()
        aircraft_type = "Unknown"
        if "waliid" in lower_filename:        # This is a hack as it's looking in the directory name and the filename and it's very chancy in any case
            aircraft_type = "Waliid"
        if "bixler" in lower_filename:        # This is a hack as it's looking in the directory name and the filename and it's very chancy in any case
            aircraft_type = "Bixler"
        if "fx-61" in lower_filename or "fx61" in lower_filename:        # This is a hack as it's looking in the directory name and the filename and it's very chancy in any case
            aircraft_type = "FX-61"
        print("======== Processing log: " + f + "========")
        log = LogFile(f, aircraft_type)
        log.AnalyzeLog()
        print("======== End file ========")
        print
        logs[f] = log

        # Write the parameter fie and print the summary
        pfile_path = f.replace(" ", "-")
        head, tail = os.path.split(pfile_path)
        create_path_if_needed(param_file_dir + head)
        f = open(param_file_dir + pfile_path, mode='w')      # Would like to replace " " with "-"
        pprint.pprint(log[keys.LOG_PARAMETERS], stream=f)
        log.PrintSummary()

f = open("log-meta-dict", mode='w')
pprint.pprint(logs, stream=f)

''' From logbook_generator:
# This array holds all the .tlog filenames in the current directory
logfiles = []
flights = []

# This iterates over the directory recursively to build a list of log files
current_directory = os.getcwd()
for (dir, _, files) in os.walk(os.getcwd()):
    for f in files:
        path = os.path.join(dir, f)
        ext = os.path.splitext(path)[1]
        if ext == ".tlog":
            logfiles.append(path)
            print path
'''
