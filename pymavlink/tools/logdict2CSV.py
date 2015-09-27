#!/usr/bin/env python

'''
Convert a log meta data file to csv format
'''

# Should log-meta-dict be a dict with redundant filename keys, or a list

import csv, ast, os
import log_meta_keys as keys

__author__ = 'kwells'

out_path = "flights.csv"

def add_stats(dict, label_prefix, stats):
    dict['flight ' + label_prefix + ' min'] = stats[keys.STATS_MIN]
    dict['flight ' + label_prefix + ' max'] = stats[keys.STATS_MAX]
    dict['flight ' + label_prefix + ' mean'] = stats[keys.STATS_MEAN]
    if keys.STATS_STDEV in stats:
        dict['flight ' + label_prefix + ' std. dev.'] = stats[keys.STATS_STDEV]

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("meta_file", metavar="LOG-META-DATA-FILE")

args = parser.parse_args()

with open(args.meta_file, 'r') as f:
    s = f.read()
    log_meta_data = ast.literal_eval(s)

log_list = []

# Flatten each flight for CSV writing
for log in log_meta_data.values():
    head, tail = os.path.split(log[keys.LOG_FILENAME])
    for flight in log[keys.LOG_FLIGHTS]:
        csv_dict = {}
        csv_dict["directory"] = head
        csv_dict["file"] = tail
        csv_dict["aircraft type"] = log[keys.LOG_AIRCRAFT_TYPE]
        csv_dict["log total time"] = log[keys.LOG_TOTAL_TIME]
        csv_dict["log record count"] = log[keys.LOG_RECORD_COUNT]
        csv_dict["log auto sections"] = log[keys.LOG_AUTONOMOUS_SECTIONS]
        csv_dict["log auto time"] = log[keys.LOG_AUTONOMOUS_TIME]
        csv_dict["log armed sections"] = log[keys.LOG_ARMED_SECTIONS]
        csv_dict["log armed time"] = log[keys.LOG_ARMED_TIME]
        # csv_dict["log total distance"] = log[keys.LOG_TOTAL_DISTANCE]  # Hack to workaround a bug...
        csv_dict["log total distance"] = log['total_diatance']
        csv_dict["mixing gain"] = log[keys.LOG_MIXING_GAIN]

        csv_dict["flight number"] = flight[keys.FLIGHT_FLIGHT_NUMBER]
        csv_dict["flight takeoffs"] = flight[keys.FLIGHT_TAKEOFF_COUNT]
        csv_dict["flight landings"] = flight[keys.FLIGHT_LANDING_COUNT]
        csv_dict["flight time"] = flight[keys.FLIGHT_FLIGHT_TIME_STRING]
        csv_dict["flight initial mode"] = flight[keys.FLIGHT_INITIAL_MODE]
        add_stats(csv_dict, "altitude", flight[keys.FLIGHT_ALTITUDE_STATS])
        add_stats(csv_dict, "airspeed", flight[keys.FLIGHT_ALTITUDE_STATS])
        add_stats(csv_dict, "throttle", flight[keys.FLIGHT_ALTITUDE_STATS])
        add_stats(csv_dict, "elevator deflection", flight[keys.FLIGHT_ALTITUDE_STATS])
        add_stats(csv_dict, "aileron deflection", flight[keys.FLIGHT_ALTITUDE_STATS])

        log_list.append(csv_dict)


# This is for pulling keys from a double layer dictionary
#fieldnames = sorted(list(set(k for d in log_meta_data.values() for k in d)))

fieldnames = sorted(list(set(k for d in log_list for k in d)))
print("Fieldnames: " + str(fieldnames))

with open(out_path, 'wb') as out_file:
    writer = csv.DictWriter(out_file, fieldnames=fieldnames, dialect='excel', quoting=csv.QUOTE_NONNUMERIC)
    writer.writeheader()
    writer.writerows(log_list)