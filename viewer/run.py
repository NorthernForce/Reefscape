from itertools import combinations
from math import atan2, pi
from time import monotonic
from cscore import CameraServer
from networktables import NetworkTables
import numpy as np
import pyrealsense2 as rs
import cv2

w, h, fps = 640, 480, 30
pipe = rs.pipeline()

meter_scale = .57 # i have trust in this goated number
colorizer = rs.colorizer()
colorizer.set_option(rs.option.visual_preset, 1)
colorizer.set_option(rs.option.min_distance, 0)
colorizer.set_option(rs.option.max_distance, meter_scale)
colorizer.set_option(rs.option.color_scheme, 3) # black to white (0->255)

config = rs.config()
config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
config.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)

hff = rs.hole_filling_filter(1) # farest from around
aligner = rs.align(rs.stream.color)

profile = pipe.start(config)

color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
depth_intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

NetworkTables.setNetworkIdentity("skynet")
NetworkTables.initialize("10.1.72.2")
viewer_nt = NetworkTables.getTable("Viewer")
cs_video = CameraServer.putVideo("Video", w, h)
cs_depth = CameraServer.putVideo("Depth", w//4, h//4)

dist_thresh = 96 # cein
time_thresh = .200 # 200 ms MAX (needs more tuning)

def find(parent, line):
    if parent[line] != line:
        return find(parent, parent[line])
    return line

def union(parent, line1, line2):
    parent[find(parent, line2)] = find(parent, line1)

# [(time, line), ...]
prev_lines = []

while True:
    start_time = monotonic()

    prev_lines = list(filter(lambda pl: start_time-pl[0] < time_thresh, prev_lines))

    frames = aligner.process(pipe.wait_for_frames())
    depth = colorizer.colorize(hff.process(frames.get_depth_frame()))
    color = frames.get_color_frame()

    color_img = cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_RGB2BGR)
    dist_img_pre = cv2.cvtColor(np.asanyarray(depth.get_data()), cv2.COLOR_RGB2GRAY)
    depth_map = np.float64(dist_img_pre)/255 * meter_scale
    dist_img = cv2.medianBlur(dist_img_pre, 17)
    
    ht, thresh_img = cv2.threshold(dist_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lt = ht * .75
    edges = cv2.Canny(dist_img, lt, ht)

    new_lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 200, 500)
    new_lines = new_lines if new_lines is not None else []
    new_lines = list(filter(lambda x: abs(atan2(x[2]-x[0], x[3]-x[1])) < pi/22, map(lambda l: tuple(l[0]), new_lines)))
    prev_lines = list(filter(lambda pl: start_time-pl[0] < time_thresh, prev_lines))
    lines = [*new_lines, *map(lambda pl: tuple(pl[1]), prev_lines)]
    prev_lines.extend(map(lambda l: (start_time, l), new_lines))

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

    offsets = list(map(lambda l: abs((l[2]+l[0])/2 - w/2), avg_lines))

    dbg_img = color_img.copy()
    dbg_img = cv2.cvtColor(dist_img, cv2.COLOR_GRAY2BGR)
    dbg_img = cv2.bitwise_or(dbg_img, cv2.merge([np.zeros_like(edges), np.zeros_like(edges), edges]))

    for x1, y1, x2, y2 in lines:
        dbg_img = cv2.line(dbg_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
    for x1, y1, x2, y2 in avg_lines:
        dbg_img = cv2.line(dbg_img, (x1, y1), (x2, y2), (0, 255, 0), 5)
    
    z_dist = float("nan")
    x_dist = float("nan")
    if avg_lines:
        x1, y1, x2, y2 = avg_lines[np.argmin(offsets)]
        midpx = (x1+x2)/2
        midpy = (y1+y2)/2

        z_dist = np.min(depth_map[
            max(int(midpy)-15, 0):min(int(midpy)+15, h),
            max(int(midpx)-15, 0):min(int(midpx)+15, w)])
        # color intrinsics because depth is mapped to color
        x_dist = z_dist * ((midpx - color_intr.ppx)/color_intr.fx)
        dbg_img = cv2.line(dbg_img, (x1, y1), (x2, y2), (255, 0, 0), 5)
        dbg_img = cv2.drawMarker(dbg_img, (int(midpx), h//2), (0, 255, 255), cv2.MARKER_CROSS, 10, 3)
        dbg_img = cv2.putText(dbg_img, f"{x_dist=:.2f} {z_dist=:.2f}", (20, 40), cv2.FONT_HERSHEY_COMPLEX, .5, (0, 0, 255), 1)
    
    end_time = monotonic()

    viewer_nt.putNumber("CandidateMetersZ", z_dist)
    viewer_nt.putNumber("CandidateMetersX", x_dist)
    viewer_nt.putNumberArray("Posts", offsets)

    dbg_img = cv2.putText(
        dbg_img, f"fps: {1/(end_time-start_time):.3f}",
        (20, 20), cv2.FONT_HERSHEY_COMPLEX, .5, (0, 0, 255), 1)
    color_img = cv2.line(color_img, (w//2, 0), (w//2, h), (0, 255, 0), 10)
    cs_video.putFrame(color_img)
    cs_depth.putFrame(dbg_img)