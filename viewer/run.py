from cscore import CameraServer
import numpy as np
import pyrealsense2 as rs
import cv2
# from ultralytics import YOLO

# if len(sys.argv) != 2:
#     print(f"usage: {sys.argv[0]} path/to/model.pt")
#     sys.exit(1)

# model = YOLO(sys.argv[1])

w, h, fps = 640, 480, 30
pipe = rs.pipeline()
colorizer = rs.colorizer()
colorizer.set_option(rs.option.visual_preset, 1)
colorizer.set_option(rs.option.min_distance, 0.25)
colorizer.set_option(rs.option.max_distance, 3)
colorizer.set_option(rs.option.color_scheme, 3) # black to white
config = rs.config()
config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
config.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)

align = rs.align(rs.stream.depth)
profile = pipe.start(config)

depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print(f"scale: {depth_scale}m")

cs_video = CameraServer.putVideo("Video", w, h)
cs_depth = CameraServer.putVideo("Depth", w, h)

while True:
    frames = align.process(pipe.wait_for_frames())
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()

    color_array = np.asanyarray(color.get_data())
    dist_array = np.asanyarray(colorizer.colorize(depth).get_data())

    print(dist_array.max(), "is max m")
    
    cs_video.putFrame(color_array)
    cs_depth.putFrame(cv2.convertScaleAbs(dist_array))