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

def find(parent, line):
    if parent[line] != line:
        return find(parent, parent[line])
    return line

def union(parent, line1, line2):
    parent[find(parent, line2)] = find(parent, line1)

while True:
    # test threshold values
    frame = (frame + 1) % 800
    dist_thresh = (frame/800) * 250

    frames = pipe.wait_for_frames()
    frames = aligner.process(frames)
    depth = colorizer.colorize(hff.process(frames.get_depth_frame()))
    color = frames.get_color_frame()

    color_img = cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_RGB2BGR)
    dist_img = cv2.cvtColor(np.asanyarray(depth.get_data()), cv2.COLOR_RGB2GRAY)
    
    ht, thresh_img = cv2.threshold(clahe.apply(dist_img), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lt = ht * .9
    edges = cv2.Canny(dist_img, lt, ht)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 300, 500)
    lines = lines if lines is not None else []
    # vertical lines ONLY (horizontal lines are the opps)
    lines = list(filter(lambda x: abs(atan2(x[2]-x[0], x[3]-x[1])) < pi/16, map(lambda l: tuple(l[0]), lines)))

    # union-find algorithm time!
    parent = dict((line, line) for line in lines)

    for line1, line2 in combinations(lines, 2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        midx1, midy1 = ((x1+x2)/2, (y1+y2)/2)
        midx2, midy2 = ((x3+x4)/2, (y3+y4)/2)
        if abs(midx2-midx1) < dist_thresh:
            union(parent, line2, line1)

    groups = {}
    for line in lines:
        rep = find(parent, line)
        if rep not in groups.keys():
            groups[rep] = list()
        groups[rep].append(line)

    avg_lines = []
    for group in groups.values():
        avg = np.uint(np.mean(group, axis=0))
        avg_lines.append(avg)

    dbg_img = color_img.copy()
    # dbg_img = cv2.addWeighted(color_img, .4, cv2.cvtColor(dist_img, cv2.COLOR_GRAY2BGR), .6, 0)
    dbg_img = cv2.bitwise_or(dbg_img, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR))
    for x1, y1, x2, y2 in avg_lines:
        dbg_img = cv2.line(dbg_img, (x1, y1), (x2, y2), (0, 255, 0), 12)
        dbg_img = cv2.line(dbg_img, (int((x1+x2)/2), int((y1+y2)/2)), (int(w/2), int((y1+y2)/2)), (255, 0, 0), 3)
    for x1, y1, x2, y2 in lines:
        dbg_img = cv2.line(dbg_img, (x1, y1), (x2, y2), (0, 0, 255), 3)

    dbg_img = cv2.putText(
        dbg_img, f"{dist_thresh=:.1f}",
        (20, 20), cv2.FONT_HERSHEY_COMPLEX, .5, (0, 0, 255), 1)
    # color = np.asanyarray(color.get_data())
    color_img = cv2.line(color_img, (w//2, 0), (w//2, h), (0, 255, 0), 10) # color when lined up good idea
    cs_video.putFrame(color_img)
    cs_depth.putFrame(dbg_img)