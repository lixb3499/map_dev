import os
import cv2
from utils import plot_one_box, cal_iou, xyxy_to_xywh, xywh_to_xyxy, updata_trace_list, draw_trace, intersect, ccw, \
     Coord_prcssing
import datetime
from matplotlib import patches
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from Map import Map
import argparse
from changelabel import changelabel
import sys
# from globle_variable import create_ax

class Logger(object):
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, "a", encoding="utf-8")

    def write(self, message):
        # self.terminal.write(message)
        self.log.write(message)

    def flush(self):
        # 必须重写此方法，因为它在某些情况下可能会被调用
        pass


def redirect_print_to_log(log_file):
    sys.stdout = Logger(log_file)


def content2detections(content, ax):
    """
    从读取的文件中解析出检测到的目标信息
    :param content: readline返回的列表
    :return: [X1, X2, ...],X1.shape = (6,)
    """
    detections = []
    for i, detection in enumerate(content):
        data = detection.replace('\n', "").split(" ")
        # detect_xywh = np.array(data[1:5], dtype="float")
        detect_xywh = np.array(data, dtype="float")
        detect_xywh = np.delete(detect_xywh, -1)

        # detect_xywh = coord_to_pixel(ax, detect_xywh)
        detections.append(detect_xywh)
    return detections


def plot_box_map(ax, box_coords):
    """
    在指定的画布上画出矩形框。

    Parameters:
        ax (matplotlib.axes._axes.Axes): Matplotlib的坐标轴对象
        box_coords (tuple): 矩形框的坐标，形式为 (x1, y1, x2, y2)
    """
    x1, y1, x2, y2 = box_coords

    # 绘制矩形框
    rect = patches.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=2, edgecolor='g', facecolor='none')

    # 将矩形框添加到坐标轴
    ax.add_patch(rect)


def main(args):
    label_path = args.data_file + "/saved_points"
    save_txt = args.save_txt
    print(args.data_file)

    # 设置视频相关参数
    SAVE_VIDEO = args.save_video
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if not os.path.exists('video_out'):
        os.mkdir('video_out')
    video_filename = os.path.join('video_out', 'exp' + current_time + '.mp4')
    log_file_name = os.path.join('log', 'exp' + current_time + '.log')
    redirect_print_to_log(log_file_name)

    frame_rate = args.frame_rate
    filelist = os.listdir(label_path)
    frame_number = len(filelist)

    # 创建Matplotlib画布和坐标轴
    fig, ax = plt.subplots(figsize=args.fig_size)

    ax.set_xlim(args.x_lim)
    ax.set_ylim(args.y_lim)
    # ax.invert_yaxis()


    areas_list = Coord_prcssing(args.coords)
    lane = args.lane
    area_pixels = []
    for area in areas_list:
        area_pixels.append([(area[i], area[i + 1]) for i in range(0, len(area), 2)])
    lane_pixels = [(lane[i], lane[i + 1]) for i in range(0, len(lane), 2)]

    # with open(os.path.join(label_path, file_name + str(0) + ".txt"), 'r', encoding='utf-8') as f:
    label_file = os.path.join(label_path, 'world_coords' + '_' + str(0).zfill(3) + ".txt")
    # content = f.readlines()
    content = changelabel(args, label_file)
    tracker = Map(content, area_pixels, lane_pixels, frame_rate)

    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    canvas = FigureCanvas(fig)

    canvas.draw()
    img_array = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
    img_array = img_array.reshape(canvas.get_width_height()[::-1] + (3,))

    frame = img_array

    # 设置视频编解码器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    if SAVE_VIDEO:
        # 创建视频写入对象
        video_writer = cv2.VideoWriter(video_filename, fourcc, frame_rate,
                                       (100 * args.fig_size[0], 100 * args.fig_size[1]))


    frame_counter = 1  # 这里由视频label文件由0还是1开始命名确定
    count1 = 0
    for frame_counter in range(1, frame_number):
        # if frame_counter % 2 == 0:
        #     # 如果 frame_counter 是偶数，跳过当前循环
        #     continue

        # fig, ax = plt.subplots(figsize=(14, 9))
        # canvas.draw()
        img_array = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        img_array = img_array.reshape(canvas.get_width_height()[::-1] + (3,))
        frame = img_array

        print(f"\r当前帧数：{frame_counter}/{frame_number}\n", end=' ')
        if frame_counter > frame_number:
            break

        label_file = os.path.join(label_path, 'world_coords' + '_' + str(frame_counter).zfill(3) + ".txt")
        content = changelabel(args, label_file)
        tracker.update(content)
        tracker.print_event()
        tracker.draw_tracks(frame)
        tracker.draw_area(frame)

        cv2.putText(frame, "ALL BOXES(Green)", (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        cv2.putText(frame, "TRACKED BOX(Red)", (25, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        for area in tracker.areas:
            cv2.putText(frame, f"count of area_{area.id}:    {area.count_car}", (25, 100 + 25 * area.id),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255),
                        2)
        cv2.putText(frame, f"{frame_counter}/{frame_number}", (25, 125 + 25 * len(tracker.areas)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow('track', frame)
        if SAVE_VIDEO:
            video_writer.write(frame)
        frame_counter = frame_counter + 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        plt.close()
    if SAVE_VIDEO:
        video_writer.release()


def parse_args():
    parser = argparse.ArgumentParser()

    # 添加参数
    parser.add_argument('--data_file', type=str, default=r'G:\jieshun\project_data\2024_6\point\172.16.1.154') # saved_point上一级
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
    parser.add_argument('--lane', nargs='+', type=float, default=[-3.2131085968935326, 18, 3.373629004435308, 0],
                        help='Define lane boundary as '
                             'x_min y_min x_max y_max')
    parser.add_argument('--lane_direction', type=int, choices=[0, 1], default=0,
                        help='Specify lane direction along x-axis(0) or y-axis(1) (default: x)')

    parser.add_argument('--coords', type=str, default=r"G:\jieshun\project_data\2024_6\point\172.16.1.154\coords.txt", help='车位坐标文件')

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    main(args)
    sys.stdout.log.close()

