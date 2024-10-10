# 使用卡尔曼滤波进行目标跟踪 - 文档更新

## 概述

提供的 Python 脚本实现了一个名为 `Tracks` 的类，使用卡尔曼滤波进行目标跟踪。该类集成了轨迹管理、IOU 匹配、状态更新、停车检测和绘图等功能，适用于多目标跟踪应用。新增功能包括停车检测、轨迹更新、速度记录以及车辆违停检测。

## 类初始化

```python
class Tracks:
    def __init__(self, initial_state, initial_cov=np.eye(6), track_id=0, frame_rate=6, licence=None, licence_cls=None):
        # 初始化各种属性和参数
        # ...
```

### 参数说明

- `initial_state`：目标的初始状态，通常为 `[x, y, w, h]` 表示目标的中心坐标和边界框的宽高。
- `initial_cov`：状态估计的初始协方差矩阵，默认值为 6x6 单位矩阵。
- `track_id`：目标的唯一标识符，用于区分多个被跟踪对象。
- `frame_rate`：视频帧率，决定了时间相关的计算。
- `licence` 和 `licence_cls`：与被跟踪对象（车辆）关联的车牌信息及其类别（适用于车辆跟踪场景）。

### 初始化内容

- `self.X`：状态向量，包含目标的坐标、尺寸及速度。
- `self.P`：状态协方差矩阵。
- `self.KF`：卡尔曼滤波器对象，用于状态预测和更新。
- `self.IOU_Threshold`：用于匹配的 IOU 阈值，默认值为 0.1。
- `self.trace_point_list`：存储目标轨迹点的列表，用于绘制轨迹。
- `self.v_average`：轨迹的平均速度。
- `self.confirmflag`：用于标记轨迹是否已确认。
- `self.parking_violation`：标记车辆是否违停。
- `self.approaching_event` 和 `self.leaving_event`：标记车辆靠近或远离参考点的事件。

## 方法

### 1. IOU 匹配

```python
def iou_match(self, detect):
    # 使用 IOU 进行匹配，更新目标框信息
    # ...
```

此方法使用 IOU（交并比）来判断目标预测框与检测框的匹配情况，并更新目标的边界框信息。

### 2. 停车检测

```python
def ifstop(self, v_Threshold):
    # 根据速度阈值判断目标是否停车
    # ...
```

- `v_Threshold`：速度阈值，如果轨迹的平均速度低于该阈值，则认为目标已停车。
- 返回值：布尔值，`True` 表示目标停车，`False` 表示未停车。

### 3. 更新目标状态

```python
def update(self, target_xywh=None):
    # 更新目标的状态和轨迹
    # ...
```

此方法使用卡尔曼滤波更新目标状态，并处理与检测到的目标框的匹配情况：

- 如果轨迹匹配成功（`id_matched=True`），则根据观测数据更新状态向量，并重新计算目标的速度。
- 如果轨迹未匹配成功，则基于历史轨迹数据进行状态预测，并更新丢失帧数。

### 4. 更新轨迹和速度

#### 更新轨迹点

```python
def update_trace_list(self, max_trace_number=50):
    # 更新轨迹点列表，存储轨迹的历史点
    # ...
```

- `max_trace_number`：存储的最大轨迹点数，超出时会移除最早的轨迹点。

#### 更新速度列表

```python
def update_v_list(self, max_v_number=5):
    # 更新速度列表，存储目标的速度向量和大小
    # ...
```

- `max_v_number`：存储的最大速度信息数量，超出时会移除最早的速度数据。

### 5. 停车时间更新

```python
def updatestoptime(self):
    # 更新目标的停车时间，如果目标被判定为停车，则增加停车时间
    # ...
```

该方法用于累积停车时间，并判断目标是否违停（超过 60 秒则标记为违停）。

### 6. 绘图

```python
def draw(self, img):
    # 在图像上绘制目标轨迹和状态信息
    # ...
```

该方法在图像上绘制目标的边界框、速度和停车状态，并区分跟踪、丢失或停车的情况。绘图信息包括：

- 目标 ID、当前速度或停车时间。
- 若目标处于停车状态，显示停车时间。
- 若目标匹配成功，绘制轨迹和速度信息。
