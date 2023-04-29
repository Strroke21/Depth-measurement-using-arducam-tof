#import essential libraries
import cv2
import depthai

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

# main loop
while True:
    # get the depth frame
    inDepth = device.getOutputQueue().get()
    depthFrame = inDepth.getFrame()

    # display the depth map
    depthMap = depthFrame / 255.0
    depthMap = cv2.applyColorMap(cv2.convertScaleAbs(depthMap, alpha=255), cv2.COLORMAP_JET)
    cv2.imshow('Depth Map', depthMap)

    # exit on ESC
    if cv2.waitKey(1) == 27:
        break

# cleanup
cv2.destroyAllWindows()
device.close()
