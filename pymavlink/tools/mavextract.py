#!/usr/bin/env python

'''
extract one mode type from a log
'''
# TODO: exclude already extracted files if we pass an argument of "*"

import sys, time, os, struct

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--mode", default='auto', help="mode to extract")
parser.add_argument("logs", metavar="LOG", nargs="+")
args = parser.parse_args()

from pymavlink import mavutil


def process(filename):
    '''process one logfile'''
    print("Processing %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps,
                                      robust_parsing=args.robust)


    ext = os.path.splitext(filename)[1]
    isbin = ext in ['.bin', '.BIN']
    islog = ext in ['.log', '.LOG']
    output = None
    count = 1
    dirname = os.path.dirname(filename)

    if isbin or islog:
        extension = "bin"
    else:
        extension = "tlog"

    file_header = ''
    last_mode = None
    selected_mode = args.mode.upper()

    while True:
        m = mlog.recv_match()
        timestamp = getattr(m, '_timestamp', None)
        if m is None:
            break
        if (isbin or islog) and m.get_type() in ["FMT", "PARM", "CMD"]:
            file_header += m.get_msgbuf()
        if (isbin or islog) and m.get_type() == 'MSG' and m.Message.startswith("Ardu"):
            file_header += m.get_msgbuf()
        # TEMPORARY: Have to remove these or else we always start the sublogs at the beginning time of the main log, 
        # and mavgraph uses that time for the start of the x-axis
        #if m.get_type() in ['PARAM_VALUE']:     #,'MISSION_ITEM']:
        #    file_header += struct.pack('>Q', timestamp*1.0e6) + m.get_msgbuf()

        if not mavutil.evaluate_condition(args.condition, mlog.messages):
            continue

        mode = mlog.flightmode.upper()
        if mode != last_mode:
            print(time.strftime("%d-%b-%Y %H:%M:%S", time.localtime(timestamp)) + ": Mode change: " + mode)
            last_mode = mode

        match = False
        if selected_mode != "ARMED":
            if mode == selected_mode:
                match = True
        else:
            if mlog.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                match = True

        if match:
            if output is None:
                path = os.path.join(dirname, "%s-%s-%u.%s" % (filename,args.mode, count, extension))
                count += 1
                print("Creating %s" % path)
                output = open(path, mode='wb')
                output.write(file_header)
        else:
            if output is not None:
                output.close()
                output = None

        if output and m.get_type() != 'BAD_DATA':
            timestamp = getattr(m, '_timestamp', None)
            if not isbin:
                output.write(struct.pack('>Q', timestamp*1.0e6))
            output.write(m.get_msgbuf())

for filename in args.logs:
    process(filename)

