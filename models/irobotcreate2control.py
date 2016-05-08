#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 5/7/2016

#for controlling the iRobot Create2
import serial
import struct
import time

irc2Connection = None
VELOCITYCHANGE = 200
ROTATIONCHANGE = 300

class IRobotCreate2Control():
   def __init_(self):
      pass
   
   def connectIRobotCreate(self, portName):
      global irc2Connection

      if irc2Connection is not None:
         print "Oops , You're already connected!"
         return

      #try:
         #ports = self.getSerialPorts()
      #   print "Select the COM port for iRobot Create\n" + '\n'.join(ports)
      #   port = raw_input()
      #except EnvironmentError:
      #   print "Enter COM port to open."
      #   port = raw_input()

      if portName is not None:
         print "Trying " + str(portName) + "... "
         try:
             irc2Connection = serial.Serial(portName, baudrate=115200, timeout=1)
             print "Connected with iRobot Create!"
         except:
             print "Connection with iRobot Create Failed."

   
   def sendCommandASCII(self, command):
      cmd = ""
      for v in command.split():
         cmd += chr(int(v))

      self.sendCommandRaw(cmd)

   def sendCommandRaw(self, command):
      global irc2Connection

      try:
         if irc2Connection is not None:
            irc2Connection.write(command)
         else:
            print "Not connected."
      except serial.SerialException:
         print "Lost irc2Connection"
         irc2Connection = None

      print ' '.join([ str(ord(c)) for c in command ])

   def getDecodedBytes(self, n, fmt):
      global irc2Connection

      try:
         return struct.unpack(fmt, irc2Connection.read(n))[0]
      except serial.SerialException:
         print "Lost connection"
         irc2Connection = None
         return None
      except struct.error:
         print "Got unexpected data from serial port."
         return None
   
   # get8Unsigned returns an 8-bit unsigned value.
   def get8Unsigned(self):
      return self.getDecodedBytes(1, "B")

    # get8Signed returns an 8-bit signed value.
   def get8Signed(self):
      return self.getDecodedBytes(1, "b")

    # get16Unsigned returns a 16-bit unsigned value.
   def get16Unsigned(self):
      return self.getDecodedBytes(2, ">H")

    # get16Signed returns a 16-bit signed value.
   def get16Signed(self):
      return self.getDecodedBytes(2, ">h")

   def sendControlKey(self, k):
      if k == 'P':   # Passive
          self.sendCommandASCII('128')
      elif k == 'S': # Safe
          self.sendCommandASCII('131')
      elif k == 'F': # Full
          self.sendCommandASCII('132')
      elif k == 'C': # Clean
          self.sendCommandASCII('135')
      elif k == 'D': # Dock
          self.sendCommandASCII('143')
      elif k == 'SPACE': # Beep
          self.sendCommandASCII('140 3 1 64 16 141 3')
      elif k == 'M': # Song
          self.sendCommandASCII('140 1 4 68 32 68 32 68 32 63 32 141 1')
      elif k == 'R': # Reset
          self.sendCommandASCII('7')
      elif k == 'UP':
          self.callbackKeyUp = True
          motionChange = True
      elif k == 'DOWN':
          self.callbackKeyDown = True
          motionChange = True
      elif k == 'LEFT':
          self.callbackKeyLeft = True
          motionChange = True
      elif k == 'RIGHT':
          self.callbackKeyRight = True
          motionChange = True
      else:
          print repr(k), "not handled"


   def sendDriveCommand(self, vr, vl):
      # create drive command
      vr = 10 * vr
      vl = 10 * vl
      cmd = struct.pack(">Bhh", 145, vr, vl)
      self.sendCommandRaw(cmd)
      
   def getEncoders(self):
      self.sendCommandASCII('149 1 43')
      time.sleep(0.1)
      leftEncoderCount = self.get16Unsigned()
      self.sendCommandASCII('149 1 44')
      time.sleep(0.1)
      rightEncoderCount = self.get16Unsigned()
      return leftEncoderCount, rightEncoderCount