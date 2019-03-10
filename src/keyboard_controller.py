#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base VideoProcessing class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; #roslib.load_manifest('ardrone_tutorials')
import rospy
import copy

# Load the DroneController class, which handles interactions with the drone, and the VideoProcessing class, which handles video display
from drone_controller import BasicDroneController
from video_processing import VideoProcessing
from diy_pid.msg import TargetCoord

# Finally the GUI libraries
from PySide import QtCore, QtGui


# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_W
	PitchBackward    = QtCore.Qt.Key.Key_S
	RollLeft         = QtCore.Qt.Key.Key_A
	RollRight        = QtCore.Qt.Key.Key_D
	YawLeft          = QtCore.Qt.Key.Key_Q
	YawRight         = QtCore.Qt.Key.Key_E
	IncreaseAltitude = QtCore.Qt.Key.Key_P
	DecreaseAltitude = QtCore.Qt.Key.Key_L
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	Emergency        = QtCore.Qt.Key.Key_Space
	Tracking		 = QtCore.Qt.Key.Key_T

# Our controller definition, note that we extend the VideoProcessing class
class KeyboardController(VideoProcessing):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		self.auto = False

		self.lastTargetX = 0
		self.lastTargetY = 0
		self.lastTargetSize = 0

		#self.subCoord = rospy.Subscriber('/target/coord',TargetCoord,self.RecieveCoord)
	
	# Proportional and derivative control to track the target, called when 'T' is pressed
	def autoTracking(self):
		while self.auto:
			self.roll = (self.centerx - self.targetX)/200.0 + (self.targetX - self.lastTargetX)/200.0
			self.lastTargetX = copy.deepcopy(self.targetX)

			self.z_velocity = (self.centery - self.targetY)/200.0 #+ (self.lastTargetY - self.targetY)/100.0
			self.lastTargetY = copy.deepcopy(self.targetX)

			self.pitch = (1000 - self.targetSize)/5000.0 - (self.lastTargetSize - self.targetSize)/100.0
			self.lastTargetSize = copy.deepcopy(self.targetSize)

			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


# We add a keyboard handler to the VideoProcessing to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Emergency:
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff:
				controller.SendTakeoff()
			elif key == KeyMapping.Land:
				controller.SendLand()
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1.5
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1.5

				elif key == KeyMapping.PitchForward:
					self.pitch += 1.5
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1.5

				elif key == KeyMapping.RollLeft:
					self.roll += 1.5
				elif key == KeyMapping.RollRight:
					self.roll += -1.5

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1.5
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1.5
				elif key == KeyMapping.Tracking:
					self.auto = not self.auto
					if self.auto:
						self.autoTracking()

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1.5
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1.5

			elif key == KeyMapping.PitchForward:
				self.pitch -= 1.5
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1.5

			elif key == KeyMapping.RollLeft:
				self.roll -= 1.5
			elif key == KeyMapping.RollRight:
				self.roll -= -1.5

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1.5
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1.5

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)



# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_keyboard_controller')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	display.show()

	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)