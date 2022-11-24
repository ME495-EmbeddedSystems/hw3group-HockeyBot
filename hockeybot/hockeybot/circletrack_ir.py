import numpy as np
import cv2
import pyrealsense2 as rs


pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_ir = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'Stereo Module':
        found_ir = True
        break
if not found_ir:
    print("The demo requires camera with IR sensor")
    exit(0)

# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 480, 270,  rs.format.y8, 90)
config.enable_stream(rs.stream.infrared, 2, 480, 270,  rs.format.y8, 90)


# Start streaming
profile = pipeline.start(config)

i =0
j =0
try:
    while True:
        frame = pipeline.wait_for_frames()
        ir_frame = frame.get_infrared_frame(1 )
        if not ir_frame:
            continue
        ir_image = np.asanyarray(ir_frame.get_data())
        circles = cv2.HoughCircles(ir_image, cv2.HOUGH_GRADIENT, 2, 70, param1=200, param2=40, minRadius=7, maxRadius=12)
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(ir_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(ir_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            # show the output image
            i += 1
            print ('i: ', i, x, y)
        cv2.namedWindow('output', cv2.WINDOW_AUTOSIZE)
        cv2.imshow("output", ir_image)
        ir_frame1 = frame.get_infrared_frame(2)
        if not ir_frame1:
            continue
        ir_image1 = np.asanyarray(ir_frame1.get_data())
        circles1 = cv2.HoughCircles(ir_image1, cv2.HOUGH_GRADIENT, 2, 70, param1=200, param2=40, minRadius=7, maxRadius=12)
        # ensure at least some circles were found
        if circles1 is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles1 = np.round(circles1[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x1, y1, r1) in circles1:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(ir_image1, (x1, y1), r, (0, 255, 0), 4)
                cv2.rectangle(ir_image1, (x1 - 5, y1 - 5), (x1 + 5, y1 + 5), (0, 128, 255), -1)
            # show the output image
            j += 1
            print ('j: ', j, x, y)
        cv2.namedWindow('output1', cv2.WINDOW_AUTOSIZE)
        cv2.imshow("output1", ir_image1)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
