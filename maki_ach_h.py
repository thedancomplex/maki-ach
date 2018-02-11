#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2018, Daniel M. Lofaro
# Copyright (c) 2018, Alberto Perez Nunez
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int32,c_int16# */
import dynamixel
import sys
import numpy as np
import time
import os
import yaml
import time
import ach
import ach


# Ach channels
MAKI_JOINT_COUNT          = 7
MAKI_CHAN_NAME_REF        = 'maki-ref'
MAKI_CHAN_NAME_STATE      = 'maki-state'

# Joint names
TSY = 0   # Torso Yaw
NKY = 1   # Neck Yaw
NKP = 2   # Neck Pitch  
EYP = 3   # Eye Pitch
EYY = 4   # Eye Yaw
ELL = 5   # Eye Lid Left
ELR = 6   # Eye Lid Right

MAKI_CHAR_PARAM_BUFFER_SIZE = 30

# Modes
MAKI_REF_MODE_FILTER    = 0 # reference to reference filter
MAKI_REF_MODE_NO_TORQUE = 1 # no torque mode


# Make Reference Structure
class MAKI_JOINT_REF(Structure):
  _pack_   = 1
  _fields_ = [
              ("ref"  , c_double),
              ("mode" , c_int16)]


class MAKI_REF(Structure):
  _pack_   = 1
  _fields_ = [
              ("joint" , MAKI_JOINT_REF*MAKI_JOINT_COUNT)]




class MAKI_JOINT_STATE(Structure):
  _pack_   = 1
  _fields_ = [
              ("pos"    , c_double), # actuial position (rad)
              ("ref"    , c_double), # last commanded reference value (rad)
              ("vel"    , c_double), # angular velocity (rad/sec)
              ("torque" , c_double), # torque on motor (A)
              ("temp"   , c_double), # Temperature (C)
              ("voltage", c_double), # System voltage (V) 
              ("name"   , c_ubyte*MAKI_CHAR_PARAM_BUFFER_SIZE)] # Space for human readiable name

class MAKI_STATE(Structure):
  _pack_   = 1
  _fields_ = [
              ("joint"   , MAKI_JOINT_STATE*MAKI_JOINT_COUNT),
              ("time"   , c_double)]



class MAKI_JOINT_PARAM(Structure):
  _pack_   = 1
  _fields_ = [
              ("id"       , c_int16),  # ID of the servo
              ("ticks"    , c_int16),  # encoder ticks per rev
              ("offset"   , c_double), # offset from center (rad)
              ("dir"      , c_double), # direction +-1 
              ("toque"    , c_double), # torque conversion (A / unit)
              ("theta_max", c_double), # max theta (rad)
              ("theta_min", c_double)] # min theta (rad)

class MAKI_PARAM(Structure):
  _pack_   = 1
  _fields_ = [
              ("joint", MAKI_JOINT_PARAM*MAKI_JOINT_COUNT),
              ("baud"     , c_int32),  # baud rate
              ("com"      , c_int16)] # COM port with dir (/dev..)




ref   = MAKI_REF()
state = MAKI_STATE()
param = MAKI_PARAM()

ref_chan   = ach.Channel(MAKI_CHAN_NAME_REF)
state_chan = ach.Channel(MAKI_CHAN_NAME_STATE)

ref_chan.flush()
state_chan.flush()


def init(com = None, baud = None):
  global ref, state, param
  if(None == com):
    com = 0

  if(None == baud):
    baud = 1000000

# setting up setting (but not using a setup file)
  for i in range(MAKI_JOINT_COUNT):
    param.joint[i].ticks     = 4096
    param.joint[i].offset    = 0.0
    param.joint[i].dir       = 1.0
    param.joint[i].torque    = 0.0045  # 4.5 mA per unit
    param.joint[i].theta_max =  np.pi/2.0 # max theta in rad
    param.joint[i].theta_min = -np.pi/2.0 # min theta in rad
    param.baud               = baud # baud rate
    param.com                = com  # com port   

  param.joint[EYY].id = 4
  param.joint[TSY].id = 30
  param.joint[NKY].id = 13
  param.joint[EYP].id = 40
  param.joint[NKP].id = 6
  param.joint[ELL].id = 5
  param.joint[ELR].id = 7

  dynSetup()



def dynSetup():
  global param
  settingsFile = 'settings.yaml'
  if os.path.exists(settingsFile):
        with open(settingsFile, 'r') as fh:
            settings = yaml.load(fh)
  else:
    settings = {}
    settings['port'] = '/dev/ttyUSB'+str(param.com)

    # Baud rate
    baudRate = param.baud
    print "##### baud = ", baudRate
    settings['baudRate'] = baudRate
ervo ID
    highestServoId = getMaxID(param)

    settings['highestServoId'] = highestServoId

    highestServoId = settings['highestServoId']

    # Establish a serial connection to the dynamixel network.
    # This usually requires a USB2Dynamixel
    serial = dynamixel.SerialStream(port=settings['port'],
                                    baudrate=settings['baudRate'],
                                    timeout=0.02)
    # Instantiate our network object
    net = dynamixel.DynamixelNetwork(serial)

    # Ping the range of servos that are attached
    print "Scanning for Dynamixels..."
    net.scan(1, highestServoId)

    settings['servoIds'] = []
    print "Found the following Dynamixels IDs: "
    for dyn in net.get_dynamixels():
        print dyn.id
        settings['servoIds'].append(dyn.id)

    # Make sure we actually found servos
    if not settings['servoIds']:
      print 'No Dynamixels Found!'
      sys.exit(0)

    # Save the output settings to a yaml file
    with open(settingsFile, 'w') as fh:
              yaml.dump(settings, fh)
    print("Your settings have been saved to 'settings.yaml'. \nTo " +
                   "change them in the future either edit that file or run " + 
                   "this example with -c.")

  mainSetup(settings)


