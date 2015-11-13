#!/usr/bin/env python

'''
Breakout MAVLink logs by flights etc.
Based on mavsummarize.py.
'''

import time, datetime, os, errno, glob, pprint, math
import csv, ast
from collections import deque

import log_meta_keys as keys

#import ../mavparam

# TODO: Looks like the first flight maximums are wrong
#   Tridge uses GPS altitude??
#   Do we need to record parameters per flight?
#   Aircraft versioning is complex...  will have to do it by date
#   How to determing mixing type?
#
# Yech - the whole structure of arming period with potentiall multiple flights is a messy legacy from wherever we got this from
#   And the level_flight is just a hack on top of that - GONE!!!
#   Refactor out the statistics collection part of Flight and then figure out what to do about the arming...

# Need to replace armed tracking time....  or at least make sure its working

# Segments
#   Record flight mode?

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

# TODO: Do we use all of these arguments?
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("logs", metavar="LOG", nargs="+", help="Log files to convert")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.mavextra import distance_two

TAKEOFF_AIRSPEED = 4.0      # meters / second
TAKEOFF_LAND_DETECTION_HYSTERESIS = 5.0     # seconds
SMOOTHING_WEIGHT = 0.97
ALT_STACK_DEPTH = 5

def TimestampString(timestamp):
    return time.strftime("%Y-%m-%d %H:%M:%S.", time.localtime(timestamp))

def TwoDec(number):
    return "{:.2f}".format(number)

def MMSSTime(time):
    return " {:4.0f}:{:02.0f}".format(int(time / 60), time % 60)     # Need to ensure a space at the beginning of the string to keep Excel from interpreting it as a time (even though it is quoted in the CSV

def create_path_if_needed(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

#class FlightState(Enum):   # Python 3 only
FS_GROUND      = 0
FS_CLIMB       = 1
FS_LEVEL       = 2
FS_DESCENT     = 3

MODE_OTHER     = 0
MODE_AUTO      = 1

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
        self.smoothed = None;

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
        if self.smoothed is None:
            self.smoothed = value
        else:
            self.smoothed = (value * (1 - SMOOTHING_WEIGHT)) + (self.smoothed * SMOOTHING_WEIGHT)


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

class FlightSegment(dict):
    def __init__(self, time, segment_number, mode, mode_string, state):
        self.altitude_stats = DataStatistics()
        self.airspeed_stats = DataStatistics()
        self.throttle_stats = DataStatistics()
        self.climb_stats    = DataStatistics()
        self.elevator_stats = DataStatistics()
        self.aileron_stats  = DataStatistics()
        self.gps_altitude_stats   = DataStatistics()
        self[keys.SEGMENT_NUMBER]           = segment_number
        self[keys.SEGMENT_FLIGHT_MODE]      = mode
        self[keys.SEGMENT_FLIGHT_MODE_STR]  = mode_string
        self[keys.SEGMENT_FLIGHT_STATE]     = state

        self.start_time     = time

    def CloseSegment(self, time):
        self[keys.SEGMENT_ALTITUDE_STATS]   = self.altitude_stats.get_stats()
        self[keys.SEGMENT_AIRSPEED_STATS]   = self.airspeed_stats.get_stats()
        self[keys.SEGMENT_THROTTLE_STATS]   = self.throttle_stats.get_stats()
        self[keys.SEGMENT_ELEVATOR_STATS]   = self.elevator_stats.get_stats()
        self[keys.SEGMENT_AILERON_STATS]    = self.aileron_stats.get_stats()
        self[keys.SEGMENT_GPS_ALT_STATS]    = self.gps_altitude_stats.get_stats()
        self[keys.SEGMENT_CLIMB_STATS]      = self.climb_stats.get_stats()

        segment_time                        = time - self.start_time
        self[keys.SEGMENT_TIME]             = segment_time
        self[keys.SEGMENT_TIME_STR]         = MMSSTime(segment_time)
        self[keys.SEGMENT_START_TIME]       = self.start_time
        self[keys.SEGMENT_START_TIME_STR]   = TimestampString(self.start_time)


# TODO: need to reexamine flight time - we seem to be accumulating total flight time in this object?? - I think that's just a leftover that needs to leave...

class Flight(dict):

    def __init__(self, flight_number):
        dict.__init__(self)
        self[keys.FLIGHT_FLIGHT_NUMBER] = flight_number
        self.flight_time = 0.0          # TODO: This is now one subtraction??
        self.takeoff_time = None
        self.landing_time = None
        self.vfr_hud_record_rate  = DataStatistics()

        # Mode
        # TODO Three are confusing - clean this up
        self.current_mode       = None          # Wierd to initialize this and not the others...
        self.current_mode_str   = None
        self.current_mode_enum  = None
        self.mode_changes       = []
        self.initial_mode       = ""            # Reexamine this

        # State
        self.flight_state       = FS_GROUND

        # Segments
        self.segments           = []
        self.current_segment    = None
        self.current_segment_num = 1

    def Takeoff(self, time, mode, mode_string, state):      # May not have been an actual detected landing - add a reason code??
        self.takeoff_time = time
        self.current_mode = mode
        self.current_mode_str = mode_string
        self.ChangeSegment(time)

    def ChangeSegment(self, time):
        if self.current_segment is not None:
            self.current_segment.CloseSegment(time)
        self.current_segment = FlightSegment(time, self.current_segment_num, self.current_mode, self.current_mode_str, self.flight_state)
        self.current_segment_num += 1
        self.segments.append(self.current_segment)      # Premature?  only do it when terminate?

    def Land(self, time):      # May not have been an actual detected landing - add a reason code??
        self.landing_time = time
        if self.landing_time is None or self.landing_time < 1230768000:
            print("Unexpected small landing time (Land method): " + str(self.landing_time))
        if self.flight_time != 0.0:
            print("Unexpected non-zero flight time: " + str(self.flight_time))
        self.flight_time = (self.landing_time - self.takeoff_time)
        self.current_segment.CloseSegment(time)

        self[keys.FLIGHT_FLIGHT_TIME]       = self.flight_time
        self[keys.FLIGHT_FLIGHT_TIME_STR]   = MMSSTime(self.flight_time)
        self[keys.FLIGHT_TAKEOFF_TIME]      = self.takeoff_time
        self[keys.FLIGHT_TAKEOFF_TIME_STR]  = TimestampString(self.takeoff_time)
        self[keys.FLIGHT_LAND_TIME]          = self.landing_time
        self[keys.FLIGHT_LAND_TIME_STR]      = TimestampString(self.landing_time)
        if self.landing_time is None or self.landing_time < 1230768000:
            print("Unexpected small landing time: " + str(self.landing_time))

        self[keys.FLIGHT_SEGMENTS]          = self.segments
        self[keys.FLIGHT_MODE_CHANGES]      = self.mode_changes
        self[keys.FLIGHT_INITIAL_MODE]      = self.initial_mode
        self[keys.FLIGHT_VFR_HUD_RECORD_RATE] = self.vfr_hud_record_rate.get_stats()

class LogFile(dict):

    # Variables declared here are CLASS variables

    def __init__(self, filename, aircraft_type):
        dict.__init__(self)

        # This is how instance variables are declared
        # Not all of these need to be intance variables
        self.flight         = None
        self.flights        = []
        self.flight_number  = 1
        self.flights_data   = []  # This is an array of arrays for numerical flight data.  Currently only used for diagnostic purposes
        self.level_segments = []
        self.level_seg_number  = 1

        self.file_name = filename

        self.autonomous_sections = 0     # How many different autonomous sections there are
        self.auto_time = 0.0             # The total time the vehicle was autonomous/guided (seconds)

        self.armed_sections = 0          # How many different armed sections there are
        self.armed_time = 0.0            # The total time the vehicle was armed (seconds)

        self.start_time = None           # The datetime of the first received message (seconds since epoch)
        self.true_time = None            # Track the first timestamp found that corresponds to a UNIX timestamp

        self.total_dist = 0.0            # The total ground distance travelled (meters)

        # Do we need to weight the cum's / counts by time delta?
        self.hud_records = []
        self.hud_count = 0

        self.total_time = 0
        self.record_count = 0;

        self.first_gps_msg   = None          # The first GPS message received
        self.last_gps_msg    = None          # The last GPS message received
        self.aircraft_type   = aircraft_type

        self.current_mode_string    = None            # This is separate from flight so we can track mode separate from the flight

        # Open the log file
        self.mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps, dialect=args.dialect)

    def start_flight(self, time, mode, state):
        print("==== Flight #" + str(self.flight_number) + " ====")
        self.flight = Flight(self.flight_number)
        self.flight.initial_mode = self.mlog.flightmode     # TODO Need to reexamine XXXXXXX
        self.flight_number += 1
        self.flights.append(self.flight)
        self.flight.Takeoff(time, mode, self.current_mode_string, state)  # TODO: hack mode_string for now - rationalize all of this mode string mess

    def AnalyzeLog(self):

        autonomous          = False         # Whether the vehicle is currently autonomous at this point in the logfile
        armed               = False         # Whether the vehicle is currently armed at this point in the logfile

        start_auto_time     = 0
        airspeed_rise_start = 0
        airspeed_drop_start = 0
        parameters          = {}  # TODO: Ultimately: MAVParmDict()
        flight_data         = None
        last_gps_raw_alt    = None
        last_timestamp      = 0.0
        last_second_timestamp  = 0.0
        altitude_samples    = deque([], maxlen = ALT_STACK_DEPTH)
        timestamp           = None
        current_mode        = None

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
            # timestamp is a float with units of seconds
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
            if m.get_type() == 'GPS_RAW_INT':       # Why are we using RAW_INT here?  And not GLOBAL_POSITION_INT

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
                if current_mode is None:
                    current_mode = m.base_mode

                # Track autonomous time - TODO Need to review all this logic now...  may be totally irrelevant...
                if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED or
                        m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and autonomous == False:
                    autonomous = True
                    self.autonomous_sections += 1
                    start_auto_time = timestamp
                elif (not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED and
                    not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and autonomous == True:
                    autonomous = False
                    self.auto_time += timestamp - start_auto_time
                    #  TODO  This is where flights used to be created,,,

                # Track armed time
                if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and armed == False:
                    armed = True
                    self.armed_sections += 1

                    # This is the logic to start a flight
                    start_armed_time = timestamp
                elif (not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and armed == True:
                    armed = False
                    self.armed_time += timestamp - start_armed_time
                    if self.flight is not None:
                        print("Landing (by end of arm)!! at: " + TimestampString(timestamp))
                        self.flight.Land(timestamp)
                        self.flight = None

                if current_mode != m.base_mode:
                    self.current_mode_string = self.mlog.flightmode         # TODO: This is just a quick hack - need to reationalize all of the mode string stuff...  And we need to detect changes in this string??
                    # Mode change
                    if (m.base_mode != 0):   # Quick hack to make it work.  Need to understand HEARTBEATS in detail
                        mode_string = ""                # TODO: this whole block is just for the debugging printf below - tear it out?
                        if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                            mode_string += "ARMED "
                        if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED):
                            mode_string += "GUIDED "
                        if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED):      # Note - this is never turned on in our logs??
                            mode_string += "AUTO "
                        #print(TimestampString(timestamp) + ": Mode changed to 0x" + format(m.base_mode, '02x') + " " +
                        #            self.mlog.flightmode + " / " + mode_string)     # Need to convert to local time here??

                        if self.flight is not None:
                            # Record mode change
                            mode_change = {}
                            # Need to use the more decoded modes
                            mode_change[keys.MODE_CHANGE_TIME] = timestamp
                            mode_change[keys.MODE_CHANGE_OLD_MODE] = self.flight.current_mode
                            mode_change[keys.MODE_CHANGE_NEW_MODE] = m.base_mode
                            mode_change[keys.MODE_CHANGE_MODE_NAME] = self.mlog.flightmode
                            self.flight.mode_changes.append(mode_change)
                            # Change flight mode and segment
                            self.flight.current_mode_str = self.current_mode_string
                            self.flight.current_mode_enum = MODE_AUTO if self.flight.current_mode_str == "AUTO" else MODE_OTHER
                            self.flight.current_mode = m.base_mode
                            # TODO: MODE and STATE should be handled in the same way - currently they are not...
                            self.flight.ChangeSegment(timestamp)         # Assumes flight is not None - which the code above does not
                        current_mode = m.base_mode


            elif m.get_type() == 'SERVO_OUTPUT_RAW' and self.flight is not None:
                self.flight.current_segment.elevator_stats.accumulate(m.servo2_raw)
                self.flight.current_segment.aileron_stats.accumulate(m.servo4_raw)

            elif m.get_type() == 'GLOBAL_POSITION_INT' and self.flight is not None:
                gps_alt = m.relative_alt / 1000.0
                self.flight.current_segment.gps_altitude_stats.accumulate(gps_alt)
                last_gps_raw_alt = gps_alt

            elif m.get_type() == 'VFR_HUD' and armed:
                if self.flight is not None:
                    # Accumulate flight statistics
                    self.flight.current_segment.altitude_stats.accumulate(m.alt)
                    self.flight.current_segment.airspeed_stats.accumulate(m.airspeed)
                    self.flight.current_segment.throttle_stats.accumulate(m.throttle)
                    self.flight.current_segment.climb_stats.accumulate(m.climb)          # TODO: Don't see these in output?
                    # should be here only for diagnostics
                    if last_timestamp != 0.0:
                        self.flight.vfr_hud_record_rate.accumulate(timestamp - last_timestamp)
                    last_timestamp = timestamp
                    if last_gps_raw_alt is not None:
                        flight_data.append([TimestampString(timestamp),
                                            self.flight.flight_state * 10,      # * 10 for visibility in the graph
                                            self.flight.current_segment[keys.SEGMENT_NUMBER],
                                            self.current_mode_string,
                                            m.alt,
                                            self.flight.current_segment.altitude_stats.smoothed,
                                            m.airspeed,
                                            self.flight.current_segment.airspeed_stats.smoothed,
                                            m.climb,
                                            self.flight.current_segment.climb_stats.smoothed,
                                            m.throttle,
                                            self.flight.current_segment.throttle_stats.smoothed,
                                            last_gps_raw_alt])
                                            #  self.flight.current_segment.gps_altitude_stats.smoothed])  TODO leave this off temporarily to avoid a None exception

                    # This detection is now different because we do it per segment - which means that every segment change with start over - probably need to add back in altitude_stats for flights
                    if timestamp - last_second_timestamp > 1.0 and self.flight.current_segment.altitude_stats.smoothed is not None:  # These initialization checks are expensive
                        altitude_samples.append(self.flight.current_segment.altitude_stats.smoothed)
                        last_second_timestamp = timestamp
                        if len(altitude_samples) == ALT_STACK_DEPTH:
                            delta = altitude_samples[ALT_STACK_DEPTH - 1] - altitude_samples[0]
                            last_flight_state = self.flight.flight_state
                            if delta > 6.0:       # Make this a constant
                                self.flight.flight_state = FS_CLIMB
                            elif delta < -6.0:
                                self.flight.flight_state = FS_DESCENT
                            else:
                                self.flight.flight_state = FS_LEVEL
                            # Flight state change
                            if last_flight_state != self.flight.flight_state:
                                self.flight.ChangeSegment(timestamp)
                                print("Flight state transition @ " + TimestampString(timestamp) + ": " + str(last_flight_state) + " to " + str(self.flight.flight_state) + ": " + str(delta))

                self.hud_records.append(m)

                # Takeoff and landing detection logic
                if self.flight is None:
                    if m.airspeed > TAKEOFF_AIRSPEED:
                        if airspeed_rise_start == 0:
                            airspeed_rise_start = timestamp
                        elif (timestamp - airspeed_rise_start) > TAKEOFF_LAND_DETECTION_HYSTERESIS:
                            print("Takeoff!! at: " + TimestampString(airspeed_rise_start))
                            self.start_flight(airspeed_rise_start, current_mode, FS_GROUND)      # This is only called from here!  FS_GROUND is already hardwired into the flight class
                            # Setup flight data collection for diagnostic purpose.  Kind of a hack for now....
                            flight_data = []
                            self.flights_data.append(flight_data)
                    else:
                        airspeed_rise_start = 0
                else:
                    if m.airspeed < TAKEOFF_AIRSPEED:
                        if airspeed_drop_start == 0:
                            airspeed_drop_start = timestamp
                        elif (timestamp - airspeed_drop_start) > TAKEOFF_LAND_DETECTION_HYSTERESIS:
                            print("Landing!! at: " + TimestampString(timestamp))
                            self.flight.Land(timestamp)
                            self.flight = None
                            flight_data = None  # Hacky!!
                            last_gps_raw_alt = None  # Hacky!!
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
        # Combined with disarm detection above
        if armed:
            self.armed_time += timestamp - start_armed_time
            if self.flight is not None:
                print("Log ended while flying!!!!: " + TimestampString(timestamp))
                self.flight.Land(timestamp)
                self.flight = None
                flight_data = None      # TODO - reexamne these 2 members...
                last_gps_raw_alt = None

        # Compute the total logtime, checking that timestamps are 2009 or newer for validity
        # (MAVLink didn't exist until 2009)
        if self.true_time:
            print("Log started at: " + TimestampString(self.true_time))
        else:
            print("Warning: No absolute timestamp found in datastream. No starting time can be provided for this log.")

        self.total_time = timestamp - self.start_time
        self[keys.LOG_PARAMETERS] = parameters

        self[keys.LOG_START_TIME]      = self.start_time
        self[keys.LOG_END_TIME]        = timestamp
        self[keys.LOG_TOTAL_TIME]      = self.total_time
        self[keys.LOG_START_TIME_STR]  = TimestampString(self.start_time)
        self[keys.LOG_END_TIME_STR]    = TimestampString(timestamp)
        self[keys.LOG_TOTAL_TIME_STR]  = MMSSTime(self.total_time)

        self[keys.LOG_RECORD_COUNT]    = self.record_count
        self[keys.LOG_AUTONOMOUS_SECTIONS]      = self.autonomous_sections
        self[keys.LOG_AUTONOMOUS_TIME] = self.auto_time
        self[keys.LOG_ARMED_SECTIONS]  = self.armed_sections
        self[keys.LOG_ARMED_TIME]      = self.armed_time
        self[keys.LOG_TOTAL_DISTANCE]  = self.total_dist
        self[keys.LOG_FILENAME]        = self.file_name

        self[keys.LOG_FLIGHTS]         = self.flights
        self[keys.LOG_LEVEL_SEGMENTS]  = self.level_segments

        # Record certain parameters
        if "VTAIL_OUTPUT" in self[keys.LOG_PARAMETERS]:
            self[keys.LOG_VTAIL_OUTPUT] = self[keys.LOG_PARAMETERS]["VTAIL_OUTPUT"]
        if "MIXING_GAIN" in self[keys.LOG_PARAMETERS]:
            self[keys.LOG_MIXING_GAIN] = self[keys.LOG_PARAMETERS]["MIXING_GAIN"]

        # Pull information from the aircraft dictionary
        if "SYSID_THISMAV" in self[keys.LOG_PARAMETERS]:
            this_mav = self[keys.LOG_PARAMETERS]["SYSID_THISMAV"]
            self[keys.LOG_SYSID] = this_mav
            if this_mav in aircraft:
                self[keys.LOG_AIRCRAFT_TYPE] = aircraft[this_mav]["type"]
                self[keys.LOG_AIRCRAFT_DESCRIPTION] = aircraft[this_mav]["description"]
                self[keys.LOG_AIRCRAFT_CONFIGURATION] = aircraft[this_mav]["configuration"]
            else:
                self[keys.LOG_AIRCRAFT_TYPE]   = self.aircraft_type

    def PrintSummary(self):
        print("=========== Summary: " + self.file_name + " ===========")
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

        #print("Flights: ")
        #pprint.pprint(self.flights)
        #print

# End of LogFile class

logs = {}
param_file_dir = "parameter-files/"
csv_file_dir = "csv-files/"

create_path_if_needed(param_file_dir)
create_path_if_needed(csv_file_dir)

### Aircraft configuration ###
aircraft_config_file = "aircraft.dict"
aircraft_config_seed_file = "aircraft-seed.dict"        # This file will be written to capture new aircraft
aircraft = {}

# Read the aircrafft configuration file
try:
    with open(aircraft_config_file, 'r') as f:
        s = f.read()
        aircraft = ast.literal_eval(s)
except IOError as exception:
    if exception.errno != errno.ENOENT:
        raise
    print(">>> No aircraft configuration file")

processed_files = 0

for filename in args.logs:
    for f in glob.glob(filename):
        lower_filename = f.lower()
        aircraft_type = "Unknown"
        if "waliid" in lower_filename:        # This is a hack as it's looking in the directory name and the filename and it's very chancy in any case
            aircraft_type = "Waliid"
        if "bixler" in lower_filename or "bix" in lower_filename:        # This is a hack as it's looking in the directory name and the filename and it's very chancy in any case
            aircraft_type = "Bixler"
        if "fx-61" in lower_filename or "fx61" in lower_filename:        # This is a hack as it's looking in the directory name and the filename and it's very chancy in any case
            aircraft_type = "FX-61"
        print
        print("======== Processing log: " + f + "========")
        log = LogFile(f, aircraft_type)
        log.AnalyzeLog()
        logs[f] = log

        # Note / TODO: The path creation has really only been tested for command line arguments of the form .../*/*.tlog
        pfile_path = f.replace(" ", "-")
        head, tail = os.path.split(pfile_path)
        head_base = os.path.basename(head)  # In case the file reference is absolute - don't create super deep directory structures

        # Write the parameter file
        create_path_if_needed(param_file_dir + head_base)
        with open(os.path.join(param_file_dir, head_base, tail + ".param"), mode='w') as f:
            pprint.pprint(log[keys.LOG_PARAMETERS], stream=f)

        # Write the data file(s)
        create_path_if_needed(csv_file_dir + head_base)
        i = 1
        for data in log.flights_data:
            ii = 1
            with open(os.path.join(csv_file_dir, head_base, tail + "-" + str(i) + ".csv"), 'w') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['record number','time', 'flight state', 'segment number', 'mode', 'alt','alt smooth','airspeed','as smooth','climb','climb smooth', \
                                 'throttle','throttle smooth','gps relative-alt','gra smooth'])
                for line in data:
                    writer.writerow([ii] + line)
                    ii += 1
            i += 1

        # Print the summary
        log.PrintSummary()
        print("======== End file ========")
        print("==========================")
        print

        today = datetime.date.today();
        tomorrow = today + datetime.timedelta(1)

        # Add any previously unknown aircraft to the aircraft dictionary (and therefore the seed file)
        if keys.LOG_SYSID in log:
            this_mav = log[keys.LOG_SYSID]
            if this_mav != 1:               # Ignore the default system ID
                if not this_mav in aircraft:
                    print("Adding new aircraft system ID: " + str(this_mav))
                    ac = {}                 # Add a default entry
                    ac["name"]          = "TBD"         # TODO make these constants
                    ac["type"]          = aircraft_type
                    ac["configuration"] = "TBD"
                    ac["description"]   = "TBD"
                    ac["owner"]         = "TBD"
                    ac["aircraft_version"]       = {str(today): "1.0", str(tomorrow): "TBD"}
                    ac["software_version"]       = {str(today): "1.0", str(tomorrow): "TBD"}
                    aircraft[this_mav] = ac
        processed_files += 1

with open("log-meta-dict.dict", mode='w') as f:
    pprint.pprint(logs, stream=f)

# Write the aircraft configuration seed file
with open(aircraft_config_seed_file, mode='w') as f:
    pprint.pprint(aircraft, stream=f, width=1)          # width=1 causes pretty printer to print one key/value per line

if processed_files == 0:
    print("mavloganalyze.py: error: no files to process: " + str(args.logs))
else:
    print("Processed " + str(processed_files))


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
