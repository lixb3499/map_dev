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
    # root = args.data_file
    # label_path = root + "/saved_points"
    # file_name = 'world_coords'
    # save_txt = 'saved_txt'  # 转换后的label文件
    #
    # # 创建Matplotlib画布和坐标轴
    # fig, ax = plt.subplots(figsize=(14, 9))
    #
    # ax.set_xlim(args.x_lim)
    # ax.set_ylim(args.y_lim)
    # # ax.invert_yaxis()
    #
    # if not os.path.exists(os.path.join(root, save_txt)):
    #     os.makedirs(os.path.join(root, save_txt))
    #
    # filelist = os.listdir(label_path)
    # n = len(filelist)
    # for i in range(n):
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

def parse_args():
    parser = argparse.ArgumentParser()

    # 添加参数
    parser.add_argument('--data_file', type=str, default=r'G:\jieshun\project_data\2024_5_13\point(1)\point\155\155_loc03\2024-05-13_11-08-58')
    # parser.add_argument('--label_path', type=str,
    #                     default=r'G:\jieshun\project_data\2024_5_13\point(1)\point\155\155_loc03\2024-05-13_11-08-58\saved_points',
    #                     help='Path to the label directory')
    parser.add_argument('--file_name', type=str, default='', help='Specify a file name')
    parser.add_argument('--save_txt', type=str, default='save_txt', help='Specify the save_txt directory')
    parser.add_argument('--save_video', action='store_true', help='Flag to save video', default=True)
    parser.add_argument('--frame_rate', type=int, default=2.605, help='Frame rate for video')
    parser.add_argument('--fig_size', nargs=2, type=float, default=[14, 9], help='Figure size as width height')
    parser.add_argument('--x_lim', nargs=2, type=float, default=[-10, 10], help='X-axis limits')
    parser.add_argument('--y_lim', nargs=2, type=float, default=[0, 20], help='Y-axis limits')
    parser.add_argument('--cls_list', type=list, default=['Car', 'Truck'], help='classes that need to be filtered out')
    parser.add_argument('--lane', nargs='+', type=float, default=[-2.5242669551176207, 18, 2.4406024322486223, 0],
                        help='Define lane boundary as '
                             'x_min y_min x_max y_max')
    parser.add_argument('--lane_direction', type=int, choices=[0, 1], default=0,
                        help='Specify lane direction along x-axis(0) or y-axis(1) (default: x)')

    # parser.add_argument('--areas', nargs='+', type=float, default=[[11.94, -2.4, 19.14, 2.4], [3, -7, 11, -1.2]],
    #                     help='Define areas as left-top and right-bottom coordinates in the format x1 y1 x2 y2, '
    #                          'x1 y1 x2 y2, ...')  # 必须是左上、右下对应的

    # parser.add_argument('--areas', nargs='+', type=float,
    #                     default=[[-6.514750679042191, 16.40145240686459, -3.514750679042191, 9.253977181370766],
    #                              [3.283867150065455, 16.40145240686459, 6.28814195563638, 9.253977181370766],
    #                              [-6.514750679042191, 7.25413595637976, -3.514750679042191, 0],
    #                              [3.283867150065455, 7.25413595637976, 6.283867150065455,0]],
    #                     help='Define areas as left-top and right-bottom coordinates in the format x1 y1 x2 y2, '
    #                          'x1 y1 x2 y2, ...')  # 必须是左上、右下对应的
    # parser.add_argument('--areas', nargs='+', type=float,
    #                     default=[[2.9138459453663663, 7.188331208487332, 8.413845945366367, 0.08731955084935972],
    #                              [2.8476686921018706, 17.64568737467815, 8.34766869210187, 8.686314766406971]],
    #                     help='Define areas as left-top and right-bottom coordinates in the format x1 y1 x2 y2, '
    #                          'x1 y1 x2 y2, ...')  # 必须是左上、右下对应的
    parser.add_argument('--coords', type=str, default=r"G:\jieshun\project_data\2024_5_13\point(1)\point\155\coords.txt", help='车位坐标文件')

    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    root = 'G:\\jieshun\\project_data\\2024_5_13\\point(1)\\point\\155\\155_loc03\\2024-05-13_11-08-58'
    label_path = root + "/saved_points"
    file_name = 'world_coords'
    i = 180
    label_path = os.path.join(label_path, file_name + '_' + str(i).zfill(3) + ".txt")
    result_content = changelabel(args, label_path)
    print(result_content)