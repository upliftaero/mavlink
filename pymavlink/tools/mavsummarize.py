#!/usr/bin/env python

'''
Summarize MAVLink logs. Useful for identifying which log is of interest in a large set.
'''

import sys, time, os, glob

# TODO: Looks like the first flight maximums are wrong

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.mavextra import distance_two


def TimestampString(timestamp):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))

def TwoDec(number):
    return "{:.2f}".format(number)

def MMSSTime(time):
    return "{:3.0f}:{:02.0f}".format(time / 60, time % 60)

# These are global so that they can be used by EndFlight()
flight_number = 1
max_altitude = 0.0
max_airspeed = 0.0

# This function is declared here so that it can access local variables
def EndFlight(time):
    global flight_number
    global max_altitude
    global max_airspeed

    flight_ID_string = "Flight #" + str(flight_number)
    print
    print flight_ID_string + " total time: " + MMSSTime(time)
    print flight_ID_string + " max altitude: " + TwoDec(max_altitude)
    print flight_ID_string + " max airspeed: " + TwoDec(max_airspeed)
    print "======== End " + flight_ID_string + " ========"
    flight_number += 1
    max_altitude = 0.0
    max_airspeed = 0.0

def PrintSummary(logfile):
    '''Calculate some interesting datapoints of the file'''
    # Open the log file
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps, dialect=args.dialect)

    autonomous_sections = 0     # How many different autonomous sections there are
    autonomous = False          # Whether the vehicle is currently autonomous at this point in the logfile
    auto_time = 0.0             # The total time the vehicle was autonomous/guided (seconds)

    armed_sections = 0          # How many different armed sections there are
    armed = False               # Whether the vehicle is currently armed at this point in the logfile
    armed_time = 0.0            # The total time the vehicle was armed (seconds)

    start_time = None           # The datetime of the first received message (seconds since epoch)
    total_dist = 0.0            # The total ground distance travelled (meters)
    last_gps_msg = None         # The first GPS message received
    first_gps_msg = None        # The last GPS message received
    true_time = None            # Track the first timestamp found that corresponds to a UNIX timestamp

    # Added state variables
    # Do we need to weight the cum's / counts by time delta?
    current_mode = None     # Track mode changes

    in_flight = False;
    global flight_number
    global max_altitude
    global max_airspeed
    max_altitude = 0.0
    max_airspeed = 0.0

    hud_records = []
    hud_count = 0

    altitude_deltas = []
    altitude_last = 0
    altitude_window = 0
    altitude_cum = 0

    airspeed_last = None
    airspeed_cum = 0
    airspeed_count = 0

    throttle_cum = 0
    throttle_count = 0

    climb_state = 0            # For now, 0 = level, 1 = climb, 2 = descent -- need to make this an enum
    vsi_state = 0

    while True:
        m = mlog.recv_match(condition=args.condition)

        # If there's no match, it means we're done processing the log.
        if m is None:
            break

        # Ignore any failed messages
        if m.get_type() == 'BAD_DATA':
            continue

        # Keep track of the latest timestamp for various calculations
        timestamp = getattr(m, '_timestamp', 0.0)

        # Log the first message time
        if start_time is None:
            start_time = timestamp

        # Log the first timestamp found that is a true timestamp. We first try
        # to get the groundstation timestamp from the log directly. If that fails
        # we try to find a message that outputs a Unix time-since-epoch.
        if true_time is None:
            if not args.notimestamps and timestamp >= 1230768000:
                true_time = timestamp
            elif 'time_unix_usec' in m.__dict__ and m.time_unix_usec >= 1230768000:
                true_time = m.time_unix_usec * 1.0e-6
            elif 'time_usec' in m.__dict__ and m.time_usec >= 1230768000:
                true_time = m.time_usec * 1.0e-6

        # Track the vehicle's speed and status
        if m.get_type() == 'GPS_RAW_INT':

            # Ignore GPS messages without a proper fix
            if m.fix_type < 3 or m.lat == 0 or m.lon == 0:
                continue

            # Log the first GPS location found, being sure to skip GPS fixes
            # that are bad (at lat/lon of 0,0)
            if first_gps_msg is None:
                first_gps_msg = m

            # Track the distance travelled, being sure to skip GPS fixes
            # that are bad (at lat/lon of 0,0)
            if last_gps_msg is not None:
                total_dist += distance_two(last_gps_msg, m)

            # Save this GPS message to do simple distance calculations with
            last_gps_msg = m

        elif m.get_type() == 'HEARTBEAT' and m.type != mavutil.mavlink.MAV_TYPE_GCS:
            # Track autonomous time
            if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED or
                m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and autonomous == False:
                autonomous = True
                autonomous_sections += 1
                start_auto_time = timestamp
            elif (not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED and
                not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and autonomous == True:
                autonomous = False
                auto_time += timestamp - start_auto_time

            # Track armed time
            if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and armed == False:
                armed = True
                armed_sections += 1
                start_armed_time = timestamp
                in_flight = True            # Just use "armed" as the flight state for now.  Will need something more ophisticated later
                start_armed_time = timestamp
                print "======== Flight #" + str(flight_number) + "========"
            elif (not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and armed == True:
                armed = False
                armed_time += timestamp - start_armed_time
                EndFlight(timestamp - start_armed_time)
                in_flight = False;

            if current_mode != m.base_mode:
                if (m.base_mode != 0):   # Quick hack to make it work.  Need to understand HEARTBEATS in detail
                    mode_string = ""
                    if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        mode_string += "ARMED "
                    if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED):
                        mode_string += "GUIDED "
                    if (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED):      # Note - this is never turned on in our logs??
                        mode_string += "AUTO "
                    print(TimestampString(timestamp) + ": Mode changed to 0x" + format(m.base_mode, '02x') + " " + 
                                mlog.flightmode + " / " + mode_string)     # Need to convert to local time here??
                    current_mode = m.base_mode

        elif m.get_type() == 'VFR_HUD':
            if m.airspeed > max_airspeed:
                max_airspeed = m.airspeed
            if m.alt > max_altitude:
                max_altitude = m.alt

            hud_records.append(m)

            # Vertical speed detection logic
            delta = (m.alt - altitude_last) if hud_count != 0 else 0
            altitude_deltas.append(delta)
            altitude_cum += m.alt
            altitude_window += delta           # Window should really be time based!!
            hud_count += 1
            altitude_last = m.alt
            if (hud_count >= 5):       # Make all of these things parameters
                altitude_window -= altitude_deltas[hud_count-4]      # Need to check these indices and boundary conditions rigorously...
                #print(TimestampString(timestamp) + ": " + "Altitudes: " + TwoDec(m.alt) + " " + TwoDec(altitude_window) + " " + TwoDec(m.climb))
                new_state = climb_state     # By default, nothing is happening
                if climb_state == 0:
                    if altitude_window > 4.0:
                        new_state = 1
                    elif altitude_window < -4.0:
                        new_state = -1
                elif climb_state == 1:
                    if altitude_window < 2.0:       # Use different thresholds to create hysterisis
                        new_state = 0
                elif climb_state == -1:
                    if altitude_window > -2.0:
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

            airspeed_last = m.airspeed
            airspeed_cum += m.airspeed
            airspeed_count += 1

            throttle_cum += m.throttle
            throttle_count += 1


    # If there were no messages processed, say so
    if start_time is None:
        print("ERROR: No messages found.")
        return

    # If the vehicle ends in autonomous mode, make sure we log the total time
    if autonomous == True:
        auto_time += timestamp - start_auto_time

    # If the vehicle ends in armed mode, make sure we log the total time
    if armed == True:
        armed_time += timestamp - start_armed_time
        EndFlight(timestamp - start_armed_time)
        in_flight = False;

    # Compute the total logtime, checking that timestamps are 2009 or newer for validity
    # (MAVLink didn't exist until 2009)
    if true_time:
        start_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(true_time))
        print("Log started at about {}".format(start_time_str))
    else:
        print("Warning: No absolute timestamp found in datastream. No starting time can be provided for this log.")

    # Print location data
    if last_gps_msg is not None:
        first_gps_position = (first_gps_msg.lat / 1e7, first_gps_msg.lon / 1e7)
        last_gps_position = (last_gps_msg.lat / 1e7, last_gps_msg.lon / 1e7)
        print("Travelled from ({0[0]}, {0[1]}) to ({1[0]}, {1[1]})".format(first_gps_position, last_gps_position))
        print("Total distance : {:0.2f}m".format(total_dist))
    else:
        print("Warning: No GPS data found, can't give position summary.")

    # Print out the rest of the results.
    total_time = timestamp - start_time
    print("Total time (mm:ss): " + MMSSTime(total_time))
    # The autonomous time should be good, as a steady HEARTBEAT is required for MAVLink operation
    print("Autonomous sections: {}".format(autonomous_sections))
    if autonomous_sections > 0:
        print("Autonomous time (mm:ss): " + MMSSTime(auto_time))

    print("Armed sections: {}".format(armed_sections))
    if armed_sections > 0:
        print("Armed time (mm:ss): " + MMSSTime(armed_time))

    print("Average airspeed: " + TwoDec(airspeed_cum / hud_count))
    print("Average altitude: " + TwoDec(altitude_cum / hud_count))
    print("Average throttle: " + TwoDec(throttle_cum / hud_count))

for filename in args.logs:
    for f in glob.glob(filename):
        print("Processing log %s" % filename)
        PrintSummary(f)