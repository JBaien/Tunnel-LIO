#!/usr/bin/python
from __future__ import print_function

import math
import optparse
import os
import sys
from xml.etree import ElementTree
import yaml

# parse the command line
usage = """usage: %prog infile.xml [outfile.yaml]

       Default output file is input file with .yaml suffix."""
parser = optparse.OptionParser(usage=usage)
options, args = parser.parse_args()

if len(args) < 1:
    parser.error('XML file name missing')
    sys.exit(9)

xmlFile = args[0]
if len(args) >= 2:
    yamlFile = args[1]
else:
    yamlFile, ext = os.path.splitext(xmlFile)
    yamlFile += '.yaml'

print('converting "' + xmlFile + '" to "' + yamlFile + '"')

calibrationGood = True
def xmlError(msg):
    'handle XML calibration error'
    global calibrationGood
    calibrationGood = False
    print('gen_calibration.py: ' + msg)

db = None
try:
    db = ElementTree.parse(xmlFile)
except IOError:
    xmlError('unable to read ' + xmlFile)
except ElementTree.ParseError:
    xmlError('XML parse failed for ' + xmlFile)

if not calibrationGood:
    sys.exit(2)

# create a dictionary to hold all relevant calibration values
calibration = {'num_lasers': 0, 'lasers': [], 'distance_resolution': 0.2}
cm2meters = 0.01                       # convert centimeters to meters
laser_index_by_id = {}
required_laser_fields = [
    'laser_id',
    'rot_correction',
    'vert_correction',
    'dist_correction',
    'dist_correction_x',
    'dist_correction_y',
    'vert_offset_correction',
    'focal_distance',
    'focal_slope',
]

def addLaserCalibration(laser_num, key, val):
    'Define key and corresponding value for laser_num'
    global calibration, laser_index_by_id
    if laser_num not in laser_index_by_id:
        laser_index_by_id[laser_num] = len(calibration['lasers'])
        calibration['lasers'].append({})
    calibration['lasers'][laser_index_by_id[laser_num]][key] = val

def enabledLaser(index):
    'Return whether index names an enabled laser from the enabled_ table.'
    if index < 0 or index >= len(enabled_lasers):
        xmlError('laser id ' + str(index) + ' outside enabled_ table')
        return False
    return enabled_lasers[index]

def validateRequiredFields():
    'Verify every enabled laser has all fields needed by calibration.cc.'
    if len(calibration['lasers']) != calibration['num_lasers']:
        xmlError('expected ' + str(calibration['num_lasers']) +
                 ' enabled laser calibration records, got ' +
                 str(len(calibration['lasers'])))
    for laser in calibration['lasers']:
        laser_id = laser.get('laser_id', 'unknown')
        for key in required_laser_fields:
            if key not in laser:
                xmlError('missing required calibration field ' + key +
                         ' for laser_id ' + str(laser_id))

# add enabled flags
num_enabled = 0
enabled_lasers = []
enabled = db.find('DB/enabled_')
if enabled == None:
    print('no enabled tags found: assuming all 64 enabled')
    num_enabled = 64
    enabled_lasers = [True for i in range(num_enabled)]
else:
    index = 0
    for el in enabled:
        if el.tag == 'item':
            this_enabled = int(el.text) != 0
            enabled_lasers.append(this_enabled)
            index += 1
            if this_enabled:
                num_enabled += 1

calibration['num_lasers'] = num_enabled
print(str(num_enabled) + ' lasers')

# add distance resolution (cm)
distLSB = db.find('DB/distLSB_')
if distLSB != None:
    calibration['distance_resolution'] = float(distLSB.text) * cm2meters

# add minimum laser intensities
minIntensities = db.find('DB/minIntensity_')
if minIntensities != None:
    index = 0
    for el in minIntensities:
        if el.tag == 'item':
            if enabledLaser(index):
                value = int(el.text)
                if value != 0:
                    addLaserCalibration(index, 'min_intensity', value)
            index += 1

# add maximum laser intensities
maxIntensities = db.find('DB/maxIntensity_')
if maxIntensities != None:
    index = 0
    for el in maxIntensities:
        if el.tag == 'item':
            if enabledLaser(index):
                value = int(el.text)
                if value != 255:
                    addLaserCalibration(index, 'max_intensity', value)
            index += 1

# add calibration information for each laser
points = db.find('DB/points_')
if points == None:
    xmlError('missing points_ calibration block')
else:
    for el in points:
        if el.tag == 'item':
            for px in el:
                index = None
                for field in px:
                    if field.tag == 'id_':
                        index = int(field.text)
                        break
                if index is None:
                    xmlError('missing required calibration field id_')
                    continue
                if not enabledLaser(index):
                    continue
                addLaserCalibration(index, 'laser_id', index)

                for field in px:
                    if field.tag == 'id_':
                        continue
                    elif field.tag == 'rotCorrection_':
                        addLaserCalibration(index, 'rot_correction',
                                            math.radians(float(field.text)))
                    elif field.tag == 'vertCorrection_':
                        addLaserCalibration(index, 'vert_correction',
                                            math.radians(float(field.text)))
                    elif field.tag == 'distCorrection_':
                        addLaserCalibration(index, 'dist_correction',
                                            float(field.text) * cm2meters)
                    elif field.tag == 'distCorrectionX_':
                        addLaserCalibration(index, 'dist_correction_x',
                                            float(field.text) * cm2meters)
                    elif field.tag == 'distCorrectionY_':
                        addLaserCalibration(index, 'dist_correction_y',
                                            float(field.text) * cm2meters)
                    elif field.tag == 'vertOffsetCorrection_':
                        addLaserCalibration(index, 'vert_offset_correction',
                                            float(field.text) * cm2meters)
                    elif field.tag == 'horizOffsetCorrection_':
                        addLaserCalibration(index, 'horiz_offset_correction',
                                            float(field.text) * cm2meters)
                    elif field.tag == 'focalDistance_':
                        addLaserCalibration(index, 'focal_distance',
                                            float(field.text) * cm2meters)
                    elif field.tag == 'focalSlope_':
                        addLaserCalibration(index, 'focal_slope', float(field.text))

# validate input data
if calibration['num_lasers'] <= 0:
    xmlError('no lasers defined')
elif calibration['num_lasers'] != num_enabled:
    xmlError('inconsistent number of lasers defined')

validateRequiredFields()

if not calibrationGood:
    sys.exit(2)

if calibrationGood:

    # write calibration data to YAML file
    f = open(yamlFile, 'w')
    try:
        yaml.dump(calibration, f)
    finally:
        f.close()
