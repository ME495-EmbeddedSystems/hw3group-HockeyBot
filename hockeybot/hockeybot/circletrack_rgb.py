# import the necessary packages
import numpy as np
import cv2
import pyrealsense2 as rs


pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires camera with Color sensor")
    exit(0)

# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

# Start streaming
profile = pipeline.start(config)

# vid = cv2.VideoCapture("NEWvid.mkv")
i =0

try:
    while True:
        frame = pipeline.wait_for_frames()
        image = frame
        output = frame
        color_frame = frame.get_color_frame()
        # Validate that both frames are valid
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # detect circles in the image
        # cv2.imshow("kjadbkjad", gray)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2, 70, param1=200, param2=40, minRadius=10, maxRadius=30)
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(color_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(color_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            # show the output image
            i += 1
            print (i, x, y)
        cv2.imshow("output", color_image)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
