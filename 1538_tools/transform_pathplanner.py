#!/usr/bin/env python3

import sys
import json
import argparse


# midpoint value in meters
MIDPOINT_X = 8.28
MIDPOINT_Y = 0.0

MIN_X = 0.03
MIN_Y = 0.0

MAX_X = 16.51
MAX_Y = 0.0

MIDPOINT_ANGLE = 90.0



if __name__ == "__main__":
    infile = sys.argv[1]
    outfile = str.replace(infile,"red","blue")


    # red to blue
    infile_data = {}
    with open(infile) as reader:
        infile_data = json.load(reader)

    print(infile_data)

    for item in infile_data["waypoints"]:
        if (item["anchor"] != 'null'):
            item["anchor"]["x"] = MIDPOINT_X - (item["anchor"]["x"] - MIDPOINT_X)
        if (item["prevControl"] != None):
            item["prevControl"]["x"] = MIDPOINT_X - (item["prevControl"]["x"] - MIDPOINT_X)
        if (item["nextControl"] != None):
            item["nextControl"]["x"] = MIDPOINT_X - (item["nextControl"]["x"] - MIDPOINT_X)
        if (item["linkedName"] != None):
            item["linkedName"] = str.replace(item["linkedName"],"red","blue")

    # rotationTargets
    for item in infile_data["rotationTargets"]:
        if (item["rotationDegrees"] != None):
            if (item["rotationDegrees"] <= 0):
                item["rotationDegrees"] = (MIDPOINT_ANGLE - (abs(item["rotationDegrees"]) - MIDPOINT_ANGLE)) * -1.0
            else:
                item["rotationDegrees"] = MIDPOINT_ANGLE - (abs(item["rotationDegrees"]) - MIDPOINT_ANGLE)
    
    # goalEndState
    if (infile_data["goalEndState"] != None):
        if (infile_data["goalEndState"]["rotation"] != None):
            if (infile_data["goalEndState"]["rotation"] <= 0):
                infile_data["goalEndState"]["rotation"] = (MIDPOINT_ANGLE - (abs(infile_data["goalEndState"]["rotation"]) - MIDPOINT_ANGLE)) * -1.0
            else:
                infile_data["goalEndState"]["rotation"] = MIDPOINT_ANGLE - (abs(infile_data["goalEndState"]["rotation"]) - MIDPOINT_ANGLE)

    # previewStartingState
    if (infile_data["previewStartingState"] != None):
        if (infile_data["previewStartingState"]["rotation"] != None):
            if (infile_data["previewStartingState"]["rotation"] <= 0):
                infile_data["previewStartingState"]["rotation"] = (MIDPOINT_ANGLE - (abs(infile_data["previewStartingState"]["rotation"]) - MIDPOINT_ANGLE)) * -1.0
            else:
                infile_data["previewStartingState"]["rotation"] = MIDPOINT_ANGLE - (abs(infile_data["previewStartingState"]["rotation"]) - MIDPOINT_ANGLE)

    print(infile_data)

    with open(outfile,'w') as writer:
        writer.write(json.dumps(infile_data,indent=2))