import irobotcreate2control
import time

#get an object of the robot Control class
irbC2 = irobotcreate2control.IRobotCreate2Control()

def irobControlTest():
   #first we establish the connection
   irbC2.connectIRobotCreate("/dev/ttyUSB0")
   
   #we reset
   irbC2.sendCommandASCII('7')
   time.sleep(5)
   #then we send a P and an S
   irbC2.sendControlKey("P")
   irbC2.sendControlKey("S")
   irbC2.sendControlKey("SPACE")

   #want to read the encoders
   #first send the read encoders opcode
   encLeft, encRight =  irbC2.getEncoders()
   print "Encoder Left: ", str(encLeft)
   print "Encoder Right: ", str(encRight)
   
   #now we move it a little
   vr = -13
   vl = -13
   irbC2.sendDriveCommand(vr, vl)
  
   time.sleep(1)
   vr = 0
   vl = 0
   irbC2.sendDriveCommand(vr, vl) #stopping.
   encLeft, encRight =  irbC2.getEncoders()
   print "Encoder Left: ", str(encLeft)
   print "Encoder Right: ", str(encRight)
   
if __name__ == "__main__":
   irobControlTest()
