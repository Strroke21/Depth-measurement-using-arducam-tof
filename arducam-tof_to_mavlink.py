import time
import depthai
from pymavlink import mavutil

# create the pipeline
pipeline = depthai.Pipeline()

# create a node for the ArduCam TOF camera
cam = pipeline.createMonoCamera()
cam.setBoardSocket(depthai.CameraBoardSocket.LEFT)
cam.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)

# create a node for the depth calculation
depth = pipeline.createStereoDepth()
depth.setConfidenceThreshold(200)
depth.setMedianFilter(depthai.MedianFilter.MEDIAN_OFF)
depth.setLeftRightCheck(False)
depth.setExtendedDisparity(False)
depth.setSubpixel(False)

# link the nodes
cam.out.link(depth.left)
depth.disparity.link(depth.rectifiedRight)
depth.depth.link(depthai.XLinkOut().input)

# create the device and start the pipeline
device = depthai.Device(pipeline)
device.startPipeline()

# connect to the drone
#specify the port where the raspberry pi is connected to flight controller 
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600) 

####################### in mission planner#############
#set proximity to mavlink
######################################################

# main loop
while True:
    # get the depth frame
    inDepth = device.getOutputQueue().get()
    depthFrame = inDepth.getFrame()

    # get the proximity measurement
    proximity = depthFrame.mean() / 1000.0  # convert to meters

    # send the proximity measurement as a MAVLink PROXIMITY message
    msg = master.mav.distance_sensor_encode(
        0,        # time_boot_ms
        mavutil.mavlink.MAV_DISTANCE_SENSOR_INFRARED,  # type
        0,        # id
        0,        # orientation
        proximity,# proximity
        0,        # covariance
        0,        # horizontal_fov
        0,        # vertical_fov
        0         # quaternion
    )
    master.mav.send(msg)

    # sleep for a short time to avoid flooding the drone with messages
    time.sleep(0.1)

# cleanup
device.close()
