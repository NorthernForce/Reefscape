# from functools import reduce, partial
from itertools import combinations
from math import atan2, hypot, pi
from cscore import CameraServer
from networktables import NetworkTables
import numpy as np
import pyrealsense2 as rs
import cv2

w, h, fps = 640, 480, 30
pipe = rs.pipeline()

meter_scale = 2.6 # i have trust in this goated number
colorizer = rs.colorizer()
colorizer.set_option(rs.option.visual_preset, 1)
colorizer.set_option(rs.option.min_distance, 0)
colorizer.set_option(rs.option.max_distance, meter_scale)
colorizer.set_option(rs.option.color_scheme, 3) # black to white

config = rs.config()
config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
config.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)

hff = rs.hole_filling_filter(1) # farest from around
clahe = cv2.createCLAHE(24, (8, 8))
aligner = rs.align(rs.stream.color)

profile = pipe.start(config)

NetworkTables.setNetworkIdentity("skynet")
NetworkTables.startClientTeam(172)
viewer_nt = NetworkTables.getTable("Viewer")

cs_video = CameraServer.putVideo("Video", w, h)
cs_depth = CameraServer.putVideo("Depth", w//4, h//4) # low because we don't need too much for debug

dist_thresh = 50
frame = 0
while True:
    # test threshold values
    frame = (frame + 1) % 500
    dist_thresh = (frame/500) * 350

    frames = pipe.wait_for_frames()
    frames = aligner.process(frames)
    depth = colorizer.colorize(hff.process(frames.get_depth_frame()))
    color = frames.get_color_frame()

    color_img = cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_RGB2BGR)
    dist_img = cv2.cvtColor(np.asanyarray(depth.get_data()), cv2.COLOR_RGB2GRAY)
    dist_img = clahe.apply(dist_img)
    
    ht, thresh_img = cv2.threshold(dist_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lt = ht * .9
    edges = cv2.Canny(dist_img, lt, ht)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 300, 500)
    lines = lines if lines is not None else []

    f_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if abs(atan2(x2-x1, y2-y1)) > pi/16: continue
        f_lines.append(line[0])

    f2_lines = []
    merged_idxs = set()
    for i in range(len(f_lines)):
        x1, y1, x2, y2 = f_lines[i]
        group = [f_lines[i]]
        for j in range(i+1, len(f_lines)):
            if j in merged_idxs:
                continue
            x3, y3, x4, y4 = f_lines[j]
            midpx1, midpy1 = ((x1 + x2)/2, (y1 + y2)/2)
            midpx2, midpy2 = ((x3 + x4)/2, (y3 + y4)/2)
            if hypot(midpx1-midpx2, midpy1-midpy2) < dist_thresh:
                merged_idxs.add(j)
                group.append(f_lines[j])
        
        group = np.array(group)
        x1avg = np.mean(group[:, 0])
        y1avg = np.mean(group[:, 1])
        x2avg = np.mean(group[:, 2])
        y2avg = np.mean(group[:, 3])
        f2_lines.append((x1avg, y1avg, x2avg, y2avg))
    
    dbg_img = color_img.copy()
    for x1, y1, x2, y2 in f2_lines:
        dbg_img = cv2.line(dbg_img, (int(x1), int(y1)), (int(x2), int(y2)), (0,0,255), 3)

    dbg_img = cv2.putText(
        dbg_img, f"{dist_thresh=:.1f}",
        (20, 20), cv2.FONT_HERSHEY_COMPLEX, .5, (0, 0, 255), 1, cv2.LINE_AA)
    
    # color = np.asanyarray(color.get_data())
    color_img = cv2.line(color_img, (w//2, 0), (w//2, h), (0, 255, 0), 10) # color when lined up good idea
    cs_video.putFrame(color_img)
    cs_depth.putFrame(dbg_img)