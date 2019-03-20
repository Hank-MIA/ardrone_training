#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; #roslib.load_manifest('ardrone_tutorials')
import rospy

# Import Image OpenCV and Numpy
import numpy as np 
import cv2 as cv

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from drone_controller import BasicDroneController

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui

# cv_bridge
from cv_bridge import CvBridge, CvBridgeError


# Some Constants
CONNECTION_CHECK_PERIOD = 50 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


class VideoProcessing(QtGui.QMainWindow):
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
		# Construct the parent class
		super(VideoProcessing, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		# Create BasicDroneController object 
		self.controller = BasicDroneController()

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo = rospy.Subscriber('/ardrone/image_raw',Image,self.ImageProcessing)
		
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
		# Target Coordinates
		self.targetX = 0
		self.targetY = 0
		# Target Size
		self.upperBound=0
		self.lowerBound=0
		self.leftBound=0
		self.rightBound=0
		self.targetSize=0
		# Target Detection
		self.detection = False

		self.bridge = CvBridge()
		self.imageLock = Lock()
		# Color range
		self.lower_red1 = np.array([0,150,100])
		self.upper_red1 = np.array([10,255,255])
		self.lower_red2 = np.array([160,150,100])
		self.upper_red2 = np.array([180,255,255])
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

		# Update the status bar to show the current drone status, battery level, and connection 
		self.statusBar().showMessage(self.statusMessage)

	def ImageProcessing(self,data):
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
			self.mask1 = cv.inRange(self.hsvImage, self.lower_red1, self.upper_red1)
			self.mask2 = cv.inRange(self.hsvImage, self.lower_red2, self.upper_red2)
			self.mask = self.mask1 + self.mask2
			# get the average (the center) of all red pixels coordinates
			ys, xs = np.where(self.mask > 0)
			if xs.size > 10:
				self.targetX = int(np.average(xs))
				self.targetY = int(np.average(ys))
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
				self.detection = True
			else:
				self.targetX = self.centerx
				self.targetY = self.centery
				self.targetSize = 1000
				self.detection = False
			self.controller.SetCoord(self.targetX, self.targetY, self.targetSize, self.detection)
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = 'Connected:{}||{}||Battery:{}%||AutoTracking:{}'.format(self.connected,msg,int(navdata.batteryPercent),self.auto)

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('Tracking')
	app = QtGui.QApplication(sys.argv)
	#controller = BasicDroneController()
	display = VideoProcessing()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
