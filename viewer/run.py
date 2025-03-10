from cscore import CameraServer
import numpy as np
import pyrealsense2 as rs
import cv2

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

pipe.start(config)

cs_video = CameraServer.putVideo("Video", w, h)
cs_depth = CameraServer.putVideo("Depth", w, h)

while True:
    frames = pipe.wait_for_frames()
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()

    color_array = np.asanyarray(color.get_data())
    dist_array = np.asanyarray(colorizer.colorize(depth).get_data())
    
    cs_video.putFrame(color_array)
    cs_depth.putFrame(cv2.convertScaleAbs(dist_array))