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
from diy_pid.msg import TargetCoord

# Import Image OpenCV and Numpy
import numpy as np 
import cv2 as cv

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui

# cv_bridge
from cv_bridge import CvBridge, CvBridgeError

# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected

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
class KeyboardController(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'

	def __init__(self):
		super(KeyboardController,self).__init__()

		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# Holds the image frame received from the drone and later processed by the GUI
		#self.image = None
		self.cvImage = None
		self.hsvImage = None
		# Image Parameters
		self.height = 0
		self.width = 0
		self.channel = 0
		self.centerx = 0
		self.centery = 0
		# Target Coordinates detected
		self.targetX = 0
		self.targetY = 0
		# Target Coordinates recieved
		self.targetx = 0
		self.targety = 0
		# Target Size
		self.upperBound = 0
		self.lowerBound = 0
		self.leftBound = 0
		self.rightBound = 0
		self.targetSize = 0

		self.bridge = CvBridge()
		self.imageLock = Lock()
		# Color range
		self.upper_green = np.array([70,255,255])
		self.lower_green = np.array([55,170,170])
		#self.lower_red1 = np.array([0,150,130])
		#self.upper_red1 = np.array([10,255,255])
		#self.lower_red2 = np.array([160,150,130])
		#self.upper_red2 = np.array([180,255,255])
		self.mask1 = None
		self.mask2 = None
		self.mask = None

		self.tags = []
		self.tagLock = Lock()
		
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		self.auto = False

		self.lastTargetX = 0
		self.lastTargetY = 0
		self.lastTargetSize = 0

		self.subCoord = rospy.Subscriber('/target/coord',TargetCoord,self.RecieveCoord)
    
	def RecieveCoord(self,targetCoord):
		if self.auto:
			self.roll = (self.centerx - targetCoord.x)/100.0 + (targetCoord.x - self.lastTargetX)/100.0
			self.lastTargetX = targetCoord.x
			self.z_velocity = (self.centery - targetCoord.y)/100.0 + (self.lastTargetY - targetCoord.y)/100.0
			self.lastTargetY = targetCoord.y
			self.pitch = (1000 - targetCoord.z)/500.0 - (self.lastTargetSize - targetCoord.z)/500.0
			self.lastTargetSize = targetCoord.z
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

    
	def RedrawCallback(self):
		if self.cvImage is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:			
					# Convert the CV image into a QImage which we can display
					image = QtGui.QPixmap.fromImage(QtGui.QImage(self.cvImage.data, self.width, self.height, self.width*3, QtGui.QImage.Format_RGB888).rgbSwapped())
					if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QtGui.QPainter()
							painter.begin(image)
							painter.setPen(QtGui.QColor(0,255,0))
							painter.setBrush(QtGui.QColor(0,255,0))
							for (x,y,d) in self.tags:
								r = QtCore.QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()
			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")     # convert ros image to BGR openCV image
			self.height, self.width, self.channel = self.cvImage.shape # get height and width of image
			self.centerx = self.width/2								   # find the center of the image
			self.centery = self.height/2
			# draw a rectangle at the center of image
			cv.rectangle(self.cvImage,(self.centerx - 10,self.centery - 10),(self.centerx + 10,self.centery + 10),(255,0,0),2)
			# creat a mask that only shows red pixels
			self.hsvImage = cv.cvtColor(self.cvImage, cv.COLOR_BGR2HSV)
			self.mask1 = cv.inRange(self.hsvImage, self.lower_green, self.upper_green)
			#self.mask2 = cv.inRange(self.hsvImage, self.lower_red2, self.upper_red2)
			#self.mask = self.mask1 + self.mask2
			# get the average (the center) of all red pixels coordinates
			ys, xs = np.where(self.mask1 > 0)
			if xs.size > 5:
				self.targetX = np.sum(xs)/xs.size
				self.targetY = np.sum(ys)/ys.size
				# draw a circle at the center of the target
				cv.circle(self.cvImage, (self.targetX,self.targetY), 2, (255,255,255),0)
				# draw a line that goes from center of the screen to the center of the target
				cv.line(self.cvImage,(self.centerx,self.centery),(self.targetX,self.targetY),(0,0,0))
				# identify the size of the target in image
				self.upperBound = np.amin(ys)
				self.lowerBound = np.amax(ys)
				self.leftBound = np.amin(xs)
				self.rightBound = np.amax(xs)
				self.targetSize = (self.lowerBound - self.upperBound)*(self.rightBound - self.leftBound)
				cv.rectangle(self.cvImage,(self.leftBound,self.upperBound),(self.rightBound,self.lowerBound),(100,100,0),2)
				# set target coordinate so it can be send by BasicDroneController (did not work, possibly has to do with class issues)
			else:
				self.targetX = self.centerx
				self.targetY = self.centery
			controller.SetCoord(self.targetX, self.targetY, self.targetSize)
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()
	
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
					if self.auto is False:
						self.pitch = 0
						self.roll = 0
						self.yaw_velocity = 0 
						self.z_velocity = 0
					print 'auto tracking is', self.auto

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
