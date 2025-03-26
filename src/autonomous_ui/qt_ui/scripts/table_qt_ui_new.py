#! /usr/bin/env python3

# from builtins import breakpoint
# from msilib.schema import Control
import rospy
import roslib; roslib.load_manifest('rviz_python_tutorial')
import sys
# import copy

import actionlib
from custom_msgs.msg import DetectionAction, DetectionActionGoal, DetectionActionResult, ToDoorAction, Bbox, Bboxes
from custom_msgs.srv import Status, StatusResponse, GetPixel
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
# from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
# from std_msgs.msg import Int8, ColorRGBA
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

from PyQt5 import QtWidgets

from PyQt5 import uic, QtGui, QtCore

# rviz
from rviz import bindings as rviz
from os.path import expanduser

# from threading import Thread, Lock

"""
https://stackoverflow.com/questions/62983277/point-cloud-distortion-when-drawing-with-modern-opengl
"""

# WARNING: check for proper absolute path
CONFIG_DIR=expanduser("~") + "/proj/rslim97/local/table-docking-roscpp/src/autonomous_ui/qt_ui/config/"

class MainWindow(QtWidgets.QMainWindow):
    imageChanged = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.simFlag = False

        # set fix main window size
        self.setWindowTitle("Table docking")
        width = 1310  # match with window size from ui file
        height = 1200
        self.setFixedSize(width,height)
        # self.setFixedWidth(width)
        # self.setFixedHeight(height)

        # self.mutex = Lock()
        # Load UI
        script_file = CONFIG_DIR + "table_docking_ui_new.ui"
        uic.loadUi(script_file, self)

        self.dockStatusText.setAlignment(QtCore.Qt.AlignCenter)

        self.bridge = CvBridge()
        # self.mousePressFlag = False
        self.mouse_click_pixel_x = None
        self.mouse_click_pixel_y = None
        self.detection_request = None
        self.table_bboxes = Bboxes()
        self.odom = Odometry()
        
        self.init_ui_()

        self.leftImageSubscriber = rospy.Subscriber("/camera_left/rgb/image_raw", Image, self.leftImageCallback, queue_size=1)
        self.rightImageSubscriber = rospy.Subscriber("/camera_right/rgb/image_raw", Image, self.rightImageCallback, queue_size=1)
        self.tableBboxesSubscriber = rospy.Subscriber("/perception/table_bboxes", Bboxes, self.tableBboxesCallback, queue_size=1)
        # subscribe to detection action server result for bounding box reset
        self.detectionActionServerSubscriber = rospy.Subscriber("/perception/detection_as/result", DetectionActionResult, self.detectionActionServerCallback, queue_size=1)
        # subscribe to current robot pose for nav goal cancellation
        self.odometrySubscriber = rospy.Subscriber("/odometry/filtered", Odometry, self.odometryCallback, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel",GoalID, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped, queue_size=1)
        # subscribe to robot current pose
        
        # Previously used to start and stop the detection with Faster-RCNN 
        self.detection_client = actionlib.SimpleActionClient('/perception/detection_as', DetectionAction)
        
        # Send clicked pixel to ui_image_interface_node. 
        self.ui_click_client = actionlib.SimpleActionClient('/ui_click_interface_as', ToDoorAction)

        # Connect buttons
        self.startDetection_button.clicked.connect(self.startDetectionClick)
        self.stopDetection_button.clicked.connect(self.stopDetectionClick)
        self.cancelAction_button.clicked.connect(self.cancelActionClick)
        self.getPixelClient = rospy.ServiceProxy("/get_pixel", GetPixel)

        self.statusServer = rospy.Service("/status", Status, self.statusCallback)
    
    def init_ui_(self):
        left_pixmap=QtGui.QPixmap('/home/rslim/proj/rslim/autonomous_ws/autonomous_ws/src/autonomous_ui/qt_ui/scripts/white.png')
        self.leftImage.setFixedSize(640,360)
        self.leftImage.setPixmap(left_pixmap)
        self.leftImage.setAlignment(QtCore.Qt.AlignCenter)
        self.leftImage.mousePressEvent = self.leftImageMousePressPixel
        # self.leftImage.mouseMoveEvent = self.leftImageMouseMovePixel
        self.leftImage.mouseReleaseEvent = self.mouseReleasePixel
        # self.topLayout.insertWidget(1, self.leftImage)

        # rightImage = QLabel(self)
        right_pixmap=QtGui.QPixmap('/home/rslim/proj/rslim/autonomous_ws/autonomous_ws/src/autonomous_ui/qt_ui/scripts/white.png')
        self.rightImage.setFixedSize(640,360)
        self.rightImage.setPixmap(right_pixmap)
        self.rightImage.setAlignment(QtCore.Qt.AlignCenter)
        self.rightImage.mousePressEvent = self.rightImageMousePressPixel
        self.rightImage.mouseReleaseEvent = self.mouseReleasePixel

        # Load Rviz for point cloud view
        leftFrame = rviz.VisualizationFrame()
        leftFrame.setSplashPath( "" )
        leftFrame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        config_file_left_view = CONFIG_DIR + "table_qt_ui_left.rviz"
        reader.readFile( config, config_file_left_view )
        leftFrame.load( config )
        leftFrame.setMenuBar( None )
        leftFrame.setStatusBar( None )
        leftFrame.setHideButtonVisibility( False )
        leftFrameManager = leftFrame.getManager()

        # Set default and current tool to click point tool
        tool_manager = leftFrameManager.getToolManager()
        setGoalTool = tool_manager.getTool(6)
        tool_manager.setCurrentTool(setGoalTool)
        tool_manager.setDefaultTool(setGoalTool)

        # Insert "leftPointCloud" widget
        ################################################
        self.middleLayout.insertWidget(1, leftFrame)

        # Load Rviz for top down view, enable after user clicks on the table
        rightFrame = rviz.VisualizationFrame()
        rightFrame.setSplashPath( "" )
        rightFrame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        config_file_top_view = CONFIG_DIR + "table_qt_ui_right.rviz"
        reader.readFile( config, config_file_top_view )
        rightFrame.load( config )
        rightFrame.setMenuBar( None )
        rightFrame.setStatusBar( None )
        rightFrame.setHideButtonVisibility( False )
        rightFrameManager = rightFrame.getManager()

        # Set default and current tool to click point tool
        tool_manager = rightFrameManager.getToolManager()
        setGoalTool = tool_manager.getTool(6)
        tool_manager.setCurrentTool(setGoalTool)
        tool_manager.setDefaultTool(setGoalTool)

        self.middleLayout.insertWidget(2, rightFrame)


    def startDetectionClick(self):
        print("Detection button clicked")
        # if self.currentMode == ToDoorGoal.DOOR_TRAVERSAL or self.currentMode == ToDoorGoal.LIFT_TRAVERSAL :
        # self.glWidget.paintGL()
        detectionActionGoal = DetectionActionGoal
        detectionActionGoal.detection_request = 'detection_button_click'
        detectionActionGoal.point_x = 0
        detectionActionGoal.point_y = 0
        self.detection_request = 'detection_button_click'
        # self.mousePressFlag = False
        self.mouse_click_pixel_x = None
        self.mouse_click_pixel_y = None
        self.detection_client.send_goal(detectionActionGoal)

    def stopDetectionClick(self):
        print("reset detection clicked")
        self.table_bboxes.bboxes = []  # reset detections
        self.detection_client.cancel_all_goals()

    def cancelActionClick(self):
        # clickPointGoal = ToDoorGoal()
        # clickPointGoal.command = ToDoorGoal().CANCEL
        # self.ui_click_client.send_goal(clickPointGoal)

        # clear arrow in rviz by publishing current robot pose
        dummy_goal = PoseStamped()
        dummy_goal.header.stamp = rospy.Time.now()
        dummy_goal.header.frame_id = "map"
        dummy_goal.pose.position = self.odom.pose.pose.position
        dummy_goal.pose.orientation = self.odom.pose.pose.orientation
        self.goal_pub.publish(dummy_goal)
        rospy.loginfo("Cleared goal in rviz")

        self.cancel_pub.publish(GoalID())
        rospy.loginfo("Sent cancel goal to move_base.")

    def leftImageCallback(self, camImage):
        # self.mutex.acquire()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(camImage, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            src = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # if self.mousePressFlag == True and self.detection_request == 'left_rgb_image_click':
            if self.detection_request == 'left_rgb_image_click':
                # plot point on image
                x = self.mouse_click_pixel_x
                y = self.mouse_click_pixel_y 
                # print("x :", x)
                # print("y :", y)
                
                cv2.circle(src, (x, y), radius=5, color=(255,255,0), thickness=2)
            # if self.detection_request == "detection_button_click":
            if len(self.table_bboxes.bboxes)>0:
                for i in range(len(self.table_bboxes.bboxes)):
                    if self.table_bboxes.bboxes[i].camera_name == "camera_left_rgb_optical_frame":
                        lt_x = self.table_bboxes.bboxes[i].top_left.x
                        lt_y = self.table_bboxes.bboxes[i].top_left.y
                        rb_x = self.table_bboxes.bboxes[i].btm_right.x
                        rb_y = self.table_bboxes.bboxes[i].btm_right.y
                        cv2.rectangle(src, (int(lt_x),int(lt_y)), (int(rb_x),int(rb_y)), color=(255,255,0), thickness=2)
                    else:
                        continue
            h, w, ch = src.shape
            bytesPerLine = ch * w
            qImg = QtGui.QImage(
                src.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888
            )
            self.imageChanged.emit(qImg)
            self.leftImage.setPixmap(QtGui.QPixmap.fromImage(qImg))

        # self.mutex.release()
    
    def rightImageCallback(self, camImage):
        # self.mutex.acquire()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(camImage, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            src = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            if self.detection_request == 'right_rgb_image_click':
                # plot point on image
                x = self.mouse_click_pixel_x
                y = self.mouse_click_pixel_y

                cv2.circle(src, (x, y), radius=5, color=(255,255,0), thickness=2)

            if len(self.table_bboxes.bboxes)>0:
                for i in range(len(self.table_bboxes.bboxes)):
                    if self.table_bboxes.bboxes[i].camera_name == "camera_right_rgb_optical_frame":
                        lt_x = self.table_bboxes.bboxes[i].top_left.x
                        lt_y = self.table_bboxes.bboxes[i].top_left.y
                        rb_x = self.table_bboxes.bboxes[i].btm_right.x
                        rb_y = self.table_bboxes.bboxes[i].btm_right.y
                        cv2.rectangle(src, (int(lt_x),int(lt_y)), (int(rb_x),int(rb_y)), color=(255,255,0), thickness=2)
                    else:
                        continue
            h, w, ch = src.shape
            bytesPerLine = ch * w
            qImg = QtGui.QImage(
                src.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888
            )
            self.imageChanged.emit(qImg)
            self.rightImage.setPixmap(QtGui.QPixmap.fromImage(qImg))
        # self.mutex.release()

    def tableBboxesCallback(self, tableBboxes):
        self.table_bboxes = Bboxes()  # reset
        # self.table_bboxes.header = tableBboxes.header
        for i in range(len(tableBboxes.bboxes)):
            self.table_bboxes.bboxes.append(tableBboxes.bboxes[i])
        # print("self.table_bboxes :",self.table_bboxes)
        return
    # def publishRGB(self):
    #     if self.rgbMsg.a != 0:
    #         self.color_publisher.publish(self.rgbMsg)

    def leftImageMousePressPixel(self, event):
        '''
        -------> x
        |  pyqt5 and opencv coordinate system
        |
        |
        v  y
        '''
        x = event.pos().x()  # x and y reversed for pyqt5
        y = event.pos().y()

        detectionActionGoal = DetectionActionGoal
        detectionActionGoal.detection_request = 'left_rgb_image_click'
        detectionActionGoal.point_x = x
        detectionActionGoal.point_y = y

        self.detection_client.send_goal(detectionActionGoal)
        self.detection_request = 'left_rgb_image_click'
        # self.mousePressFlag = True
        self.mouse_click_pixel_x = x
        self.mouse_click_pixel_y = y
        # self.currentPressPixel = event.pos()

    def rightImageMousePressPixel(self, event):
        x = event.pos().x()  # x and y reversed for pyqt5
        y = event.pos().y()

        detectionActionGoal = DetectionActionGoal
        detectionActionGoal.detection_request = 'right_rgb_image_click'
        detectionActionGoal.point_x = x
        detectionActionGoal.point_y = y

        self.detection_client.send_goal(detectionActionGoal)
        self.detection_request = 'right_rgb_image_click'
        self.mouse_click_pixel_x = x  # x and y reversed
        self.mouse_click_pixel_y = y

    def mouseReleasePixel(self, event):
        print("release pixel:")
        print(event.pos())


    def statusCallback(self, req):
        self.dockStatusText.setText("Current Status: " + req.status)
        currentResponse = StatusResponse()
        currentResponse.received = True
        return currentResponse
        # print(dockingState[statusId.data])

    def detectionActionServerCallback(self, msg):
        # cancel goal and clear goal in rviz
        # reset bounding boxes
        if msg:
            time.sleep(3.0)
            self.table_bboxes = Bboxes()  # reset
        
    def odometryCallback(self, msg):
        self.odom = Odometry()
        self.odom.header.stamp = msg.header.stamp
        self.odom.header.frame_id = msg.header.frame_id
        self.odom.child_frame_id = msg.child_frame_id
        self.odom.pose = msg.pose

if __name__ == "__main__":
    rospy.init_node('pyqt_view')
    rospy.loginfo("Node initialised")

    app = QtWidgets.QApplication( sys.argv )
    myviz = MainWindow()
    myviz.show()

    sys.exit(app.exec_())

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        myviz.publishRGB()
        rate.sleep()    