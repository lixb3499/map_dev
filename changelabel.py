import os
import numpy as np
from globle_variable import ax
import argparse

def coord_to_pixel(ax, coord):
    """
    将坐标中的点映射到画布中的像素点。

    Parameters:
        ax (matplotlib.axes._axes.Axes): Matplotlib的坐标轴对象
        coord (tuple): 坐标中的点，形式为 (x, y)

    Returns:
        tuple: 画布中的像素点，形式为 (pixel_x, pixel_y)
    """
    x, y = coord
    pixel_x, pixel_y = ax.transData.transform_point((x, y))

    # # 反转y轴
    pixel_y = 900 - pixel_y

    return int(pixel_x), int(pixel_y)


# # 设置视频相关参数
# video_filename = 'output_video.mp4'
# frame_rate = 6
# # duration = 10  # 视频时长（秒）

def content2detections(content, ax, cls_list, lane_direct, range=(-4, 4)):
    """
    从读取的文件中解析出检测到的目标信息
    :param content: readline返回的列表
    :param ax: 创建的画布，我们需要将地图坐标点转化为像素坐标
    :return: [X1, X2]
    """
    detections = []
    for i, detection in enumerate(content):
        data = detection.replace('\n', "").split(" ")
        # detect_xywh = np.array(data[1:5], dtype="float")
        if data[2] in cls_list:
            detect_xywh = np.array(data[0:2], dtype="float")
            if len(detect_xywh) != 2:  # 有时候给到的数据是10.874272061490796 3.172816342766715 0.0形式的
                detect_xywh = np.delete(detect_xywh, -1)
            if min(range) < detect_xywh[lane_direct] < max(range):
                # detect_xywh = coord_to_pixel(ax, detect_xywh)
                detections.append([detect_xywh, *data[2:]])
    return detections

def changelabel(args, file):
    result_content = []
    with open(file, 'r', encoding='utf-8') as f:
        content = f.readlines()
        detection = content2detections(content, ax, args.cls_list, args.lane_direction, (args.lane[args.lane_direction]-0.08, args.lane[args.lane_direction+2]+0.08))
        # detection = content2detections(content, ax, args.cls_list, args.lane_direction, (-4, 4))
        for item in detection:
            coordinates_str = ' '.join(map(str, item[0]))
            item[0] = coordinates_str

            # 将所有数据转换为字符串，例如：'795 449 4 粤BG53030 新能源牌'
            data_str = ' '.join(map(str, item))+'\n'

            result_content.append(data_str)
    return  result_content

