# 目标跟踪与车辆行为判断

## 复现步骤

### 环境安装

我们可以直接使用检测组YOLOv8的环境，如果你不想知道检测组干了些啥，你可以用下面的命令安装所需的依赖库：

```shell
pip install -r requirements.txt
```

我使用的python版本为3.9.18，版本不符合可能会造成不可预料的问题。

### Update（8月12）: 简化复现操作

当前的复现步骤：

我们拿到的数据文件夹长这样：

```
172.16.1.155:.
│  106.mp4
│  coords.txt
│  exp2024-08-05_21-44-34.mp4
│  track_yolov8_20240803_120356.avi
│
└─saved_points
        world_coords_000.txt
        world_coords_001.txt
        world_coords_002.txt
        world_coords_003.txt
        ...
```

对于每一个视频，需要更改相应的车辆投影后的点坐标文件以及车位的坐标文件。具体来说，我么们需要在parse_args()函数中更改：

1、车辆投影后的点坐标文件，但是不需要到saved_points。

```python
parser.add_argument('--data_file', type=str, default=r'G:\jieshun\project_data\8_5test\point - 副本\172.16.1.155') # saved_points上一级
```

2、车位的坐标文件，就是上面的coords.txt

```python
parser.add_argument('--coords', type=str, default=r"G:\jieshun\project_data\8_5test\point - 副本\172.16.1.155\coords.txt", help='车位坐标文件')
```

3、车道坐标：如下所示，18和0是不需要改的，-2.8580322011818176表示车道左边界，3.9289563490177057表示车道右边界，这两个需要根据coords.txt里面的内容填写

```python
parser.add_argument('--lane', nargs='+', type=float, default=[-2.8580322011818176, 18, 3.9289563490177057, 0],
                    help='Define lane boundary as '
                         'x_min y_min x_max y_max')
```

4、运行，log文件夹里面有log，video_out文件夹里面有输出视频。

### 更多数据

你可以在下面的链接中得到我们更多的demo使用的数据，更改相应的路径即可。

数据链接：
链接：https://pan.baidu.com/s/1tSKmXOk_jpk7Gvc8n4N8Ag 
提取码：qjck 
