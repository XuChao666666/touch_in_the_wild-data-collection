#!/usr/bin/env python3
"""ROS 2 publishers for left and right tactile sensors."""

import time
import threading
from typing import List

import numpy as np
import serial
from scipy.ndimage import gaussian_filter

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rummy_tactile_msgs.msg import TactileInput

THRESHOLD   = 15    # 触觉压力阈值
NOISE_SCALE = 50    # 噪声归一化系数
BAUD        = 2_000_000     # 串口波特率

'''
    辅助函数：_temporal_filter
        功能：实现时间域上的低通滤波，减少数据抖动（类似EMA算法）
        参数：alpha=0.5表示新旧数据权重各占50%
'''
def _temporal_filter(new: np.ndarray, prev: np.ndarray, alpha: float = 0.5) -> np.ndarray:
    """Simple exponential smoothing. 指数平滑滤波"""
    return alpha * new + (1 - alpha) * prev


class TactileSensorNodeLeft(Node):
    def __init__(self) -> None:     # -> None 没有返回值
        super().__init__("tactile_sensor_node_left")     # "tactile_sensor_node_left"是传递给父类的参数，这里父类继承自Node，表示的是节点的名称；
        self._pub = self.create_publisher(TactileInput, "tactile_input_left", 10)    # 创建发布者；参数：TactileInput：要发布的消息类型，在自定义的msg文件中，表示发布者发布的特定结构的消息
                                                                                               # 参数："tactile_input_left" 话题名称
                                                                                               # 参数：10 消息队列大小，当发送消息的速度超过处理速度的时候ROS最多缓存10条数据，超过这个数量的数据之后，新消息会覆盖旧消息（防止溢出）

        self.declare_parameter("tactile_sensor_name_left", "left_finger_real")  #  声明节点参数：在当前节点中声明一个名为"tactile_sensor_name_left"的参数，并设置默认值为"left_finger_real"
                                                                                # 参数的主要用途：允许在节点运行时动态配置参数值（无需修改代码），比如通过启动文件（.launch.py）、命令行或 ROS 2 参数服务进行配置。
        dev = self.get_parameter("tactile_sensor_name_left").value  # 获取参数值：tactile_sensor_name_left 对应的参数值
        self.get_logger().info(f"[LEFT] /dev/{dev}")     # 获取当前节点的日志信息；info表示信息级别的日志，表示提示常规信息；f是常见的格式化字符；[LEFT] 是一个标识，通常用于区分不同设备或模块（这里表示 "左侧" 设备）；{dev}一个变量占位符，会被变量 dev 的实际值替换；

        try:
            self.ser = serial.Serial(f"/dev/{dev}", BAUD) # 初始化串口通信
        except serial.SerialException as err:
            self.get_logger().error(f"[LEFT] {err}") # 若打开失败，在日志中输出错误
            raise

        self._prev = np.zeros((16, 32))     # 初始化类的成员变量self._prev（下划线私有变量，仅在类的内部使用），将其赋值为16 行，32 列的一个全为 0 的数组（矩阵）
        self._median: np.ndarray | None = None  # 声明一个成员变量 self._median，并通过类型提示（type hint）指定它的可能类型；np.ndarray 表示该变量可以是 NumPy 数组，| None 表示该变量也可以是 None（空值）。= None为变量设置初始值为 None，表示在初始化时该变量处于"未赋值"状态
        self._last_pub = time.time() # 声明一个类的成员变量为当前时间

        threading.Thread(target=self._reader, daemon=True).start()  # 创建并启动一个线程，用于异步执行指定的任务（通常是持续读取数据的操作）。
                                                                    # target=self._reader 表示线程将执行当前类中的 _reader 方法。
                                                                    # daemon=True： daemon 参数设置线程为 "守护线程"（后台线程）。特性：当主线程结束时，守护线程会被强制终止（不会阻塞程序退出）；适合用于持续运行的辅助任务（如数据读取），避免主线程结束后这些线程还在后台运行；
                                                                    # .start(): 调用线程对象的 start() 方法，正式启动线程;启动后会自动执行 target 指定的 self._reader 方法;
    '''
        定义了一个静态方法，用于对数组进行高斯模糊处理。

    '''
    @staticmethod   # 静态方法装饰器，表明该方法是类的静态方法。
    def _blur(arr: np.ndarray, sigma: float = 0.1) -> np.ndarray:  # blur意思是模糊；返回值是np.ndarray
        return gaussian_filter(arr, sigma=sigma)    # 调用了 SciPy 库中的gaussian_filter函数（高斯滤波函数）；该函数使用高斯核对输入数组arr进行卷积运算，实现平滑模糊效果；将参数sigma传递给滤波函数，控制模糊程度；

    def _reader(self) -> None:  # 读取线程，且该方法没有返回值；
        samples: List[np.ndarray] = []  # 声明一个名为 samples 的列表，类型提示 List[np.ndarray] 表明它是一个存储 NumPy 数组 的列表。
        block:   List[List[int]]  = []  # 声明一个名为 block 的列表，类型提示 List[List[int]] 表明它是一个 “整数列表的列表”（二维列表）。
        t0 = 0.0

        while True:
            if self.ser.in_waiting == 0:    # 如果缓冲区为空，跳过本次循环
                continue
            line = self.ser.readline().decode(errors="ignore").strip()  #readline()读取整行数据，返回字节流；decode将字节流解码为字符串（errors="ignore" 忽略解码失败的字符）；.strip()去除字符串收尾的空白字符，得到干净的一行数据；
            if len(line) < 10:  # 若当前一行长度小于10，则检查之前积累的block是否完整；
                if len(block) == 16 and all(len(r) == 32 for r in block):   # 若block符合条件（16行，每行32个元素），则存入samples
                    samples.append(np.array(block))
                    fps = 1 / (time.time() - t0) if t0 else 0.0     # 计算帧率（FPS）：当前时间与上一次存储的时间差的倒数
                    self.get_logger().debug(f"[LEFT] fps: {fps:.2f}")   # 输出调试日志
                    if len(samples) > 30:
                        break
                block, t0 = [], time.time()     # 重置block和时间戳，准备接收下一个数据块
                continue
            block.append([int(v) for v in line.split()])   # 如果line长度≥10，则有line.split()：按空白字符分割字符串，得到一个字符串列表（如 "1 2 3" 分割为 ["1", "2", "3"]）。int(v)将每个字符串转换为整数；block.append(...)：将这行整数列表存入 block，积累为数据块的一部分。

        self._median = np.median(np.array(samples), axis=0)     # sample是之前采集的传感器数据列表（存储了 30 + 个有效数据帧，每个帧是 16×32 的 NumPy 数组）。axis=0 表示沿着 “样本帧” 维度（第一个维度）计算中位数。
        self.get_logger().info("[LEFT] calibration complete")

        block, latest = [], None
        while rclpy.ok():   # 正式运行阶段；rclpy.ok() 表示节点正常状态下持续运行；
            if self.ser.in_waiting == 0:
                continue
            line = self.ser.readline().decode(errors="ignore").strip()  # 同上
            if len(line) < 10:
                if len(block) == 16:
                    latest = np.array(block)    # 当line的长度短的时候，处理之前的block；
                block = []
                if latest is not None:
                    frame = np.clip(latest - self._median - THRESHOLD, 0, 100)  # 当前数据减去校准阶段得到的中位数基准，再减去阈值（过滤微小噪声）；np.clip将处理后的数据限制在 0~100 范围内（超出部分截断）
                    norm  = frame / (NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame))   # 数据归一化处理
                    filt  = _temporal_filter(norm, self._prev); self._prev = filt   # 调用时间域滤波函数，结合当前帧 norm 和上一帧数据 self._prev 进行平滑处理（减少数据抖动）。
                                                                                    # self._prev = filt：将当前滤波后的结果存入 self._prev，作为下一帧处理的历史参考。

                    msg = TactileInput()    # 创建 TactileInput 类型的消息对象
                    msg.header.stamp = self.get_clock().now().to_msg()  # 为消息添加 ROS 2 系统时间戳（通过节点时钟获取当前时间），用于同步不同节点的消息。
                    msg.local_time   = str(time.time())    # 将当前系统时间（浮点数）转换为字符串，存入消息的 local_time 字段
                    msg.data = filt[4:].flatten().astype(float).tolist()    # filt[4:]：取滤波后数据从第 4 行开始的部分；.flatten()：将二维数组（16×32）展平为一维数组；.astype(float)：转换为浮点型；.tolist()：转换为 Python 列表（符合 ROS 2 消息对数组类型的要求）；
                    self._pub.publish(msg)  # 通过之前创建的发布者 self._pub，将消息发布到 tactile_input_left 话题（供其他节点订阅）。
                    self._last_pub = time.time()    # 存储当前时间戳，记录最后一次发布时间；
                continue
            block.append([int(v) for v in line.split()])

# 同上
class TactileSensorNodeRight(Node):
    def __init__(self) -> None:
        super().__init__("tactile_sensor_node_right")
        self._pub = self.create_publisher(TactileInput, "tactile_input_right", 10)

        self.declare_parameter("tactile_sensor_name_right", "right_finger")
        dev = self.get_parameter("tactile_sensor_name_right").value
        self.get_logger().info(f"[RIGHT] /dev/{dev}")

        try:
            self.ser = serial.Serial(f"/dev/{dev}", BAUD)
        except serial.SerialException as err:
            self.get_logger().error(f"[RIGHT] {err}")
            raise

        self._prev = np.zeros((16, 32))
        self._median: np.ndarray | None = None
        self._last_pub = time.time()

        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def _blur(arr: np.ndarray, sigma: float = 0.1) -> np.ndarray:
        return gaussian_filter(arr, sigma=sigma)

    def _reader(self) -> None:
        samples: List[np.ndarray] = []
        block:   List[List[int]]  = []
        t0 = 0.0

        while True:
            if self.ser.in_waiting == 0:
                continue
            line = self.ser.readline().decode(errors="ignore").strip()
            if len(line) < 10:
                if len(block) == 16 and all(len(r) == 32 for r in block):
                    samples.append(np.array(block))
                    fps = 1 / (time.time() - t0) if t0 else 0.0
                    self.get_logger().debug(f"[RIGHT] fps: {fps:.2f}")
                    if len(samples) > 30:
                        break
                block, t0 = [], time.time()
                continue
            block.append([int(v) for v in line.split()])

        self._median = np.median(np.array(samples), axis=0)
        self.get_logger().info("[RIGHT] calibration complete")

        block, latest = [], None
        while rclpy.ok():
            if self.ser.in_waiting == 0:
                continue
            line = self.ser.readline().decode(errors="ignore").strip()
            if len(line) < 10:
                if len(block) == 16:
                    latest = np.array(block)
                block = []
                if latest is not None:
                    frame = np.clip(latest - self._median - THRESHOLD, 0, 100)
                    norm  = frame / (NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame))
                    filt  = _temporal_filter(norm, self._prev); self._prev = filt

                    msg = TactileInput()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.local_time   = str(time.time())
                    msg.data = filt[4:].flatten().astype(float).tolist()
                    self._pub.publish(msg)
                    self._last_pub = time.time()
                continue
            block.append([int(v) for v in line.split()])


def main(args=None) -> None:
    rclpy.init(args=args)   # ROS2 初始化
    executor = MultiThreadedExecutor(num_threads=2)     # 创建多线程执行器
    executor.add_node(TactileSensorNodeLeft())   # 添加节点到执行器（左侧传感器）
    executor.add_node(TactileSensorNodeRight())     # 添加节点到执行器（右侧传感器）
    try:
        executor.spin()     # 启动一个持续运行的循环
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
