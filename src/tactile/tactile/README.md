# README
tactile 文件夹中存放数据收集的核心代码：
- sensor 和 Viewer
  - 在这里的sensor之前有几版不同的版本，下面想要做的是基于当前的触觉效果做出一个宽的，高保真还原的触觉。那这个新的就叫tactile_sensor_new.py
  - 但是在这之前，要先搞一个tactile_sensor_debug.py 看看读取的数据格式是什么样的，然后再做下一步的处理。

# tactile_viewer_debug.py
---
## TactileSensorNodeRight 核心逻辑
我先分析了这段代码的执行逻辑：
### 触觉数据采集逻辑 TactileSensorNodeRight类
#### init 函数
1. 创建publisher：发布 TactileInput 到 tactile_input_right
2. 打开串口：/dev/{dev}，波特率 BAUD
3. 起一个守护线程跑 _reader()：
   ```
    threading.Thread(target=self._reader, daemon=True).start()
   ```
    所以真正读传感器数据的逻辑全部在 _reader()，而且它不依赖 ROS 的 spin 回调，线程自己一直跑。


#### _reader 函数
block 是“当前拼的这一帧”，类型是`List[List[int]]`
每次读到一整行，就把这一行的转成int列表塞进去：
```
block.append([int(v) for v in line.split()])
```
##### 那什么时候认为一帧结束了呢？
当读到一行长度小于10的时候就认为一行结束。
然后检查block是否完整：一共有16行，一行有16个数
之后再把这一帧保存到samples中：`samples.append(np.array(block))`
当sample的数量大于30，就break；

#### fps 表示什么？
fps表示每次存放一帧，就用当前时间减去上一次帧结束的时间戳`t_0`来估算FPS

#### _median 是什么？
- samples 形状大概是 (31, 16, 16)
- axis=0 表示对“帧维度”取中位数
- 得到 _median 形状 (16, 16)
你有 N 张 16×16 的帧（N≈31），把它们叠起来变成一个 N×16×16 的三维数组，然后对 每一个像素位置 (i,j) 在 N 个帧里的数值做中位数。

### 运行阶段（running stage）：不断拼帧->去基线->阈值->归一化->时间滤波 -> 旋转 -> 发布
block 和 latest 两帧：
- block 还是正在拼的这一帧
- latest 上一帧拼好的np.array(block)（缓存一帧用于处理）
逻辑还是：
- 读到长行就是block.append
- 读到短行就是认为帧结束
这里不检查每行是不是16列，只检查16行

#### 当latest is not None 的时候：
1. 去基线 + 去阈值 + 截断范围
```
frame = np.clip(latest - self._median - THRESHOLD, 0, 100)
```
含义：
- latest - self._median：去掉静态背景（校准值）
- 再减 THRESHOLD：把小于阈值的触碰当成噪声（压下去）
- np.clip(..., 0, 100)：最终限定在 0~100，负数变0，过大变100
得到的frame 仍是16*16（理想情况下）

2. 归一化到 0~1 左右
这里要解释一行代码，感觉这里设置的还是比较巧妙的：
```
denom = NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame)
norm  = frame / denom
```
在这行代码中，目的是要实现归一化，他的处理方式是：
   1. 找到frame一帧中的最大值，看这个值和THRESHOLD 的相对大小，如果小于THRESHOLD：
      1. 如果这个值是很小的，也就是认为他是类似于噪声的感觉
      2. 如果你用他的最大值作为分母去归一化他的话，就会出现一种情况，就是归一化结果会很大，因为是相对于一帧来说的
      3. 所以下面要做的事情是如果这个值很小，就要给他配一个比较大的分母，这样可以保证归一化后的结果也可以正确反映力很弱的现实。
   2. 如果大于THRESHOLD是什么样的结果：
      1. 就正常缩放就可以了。当前帧最大值为1，其他值分布在0和1之间。

3. 时间滤波（指数平滑 / EMA）
```
filt = _temporal_filter(norm, self._prev)
self._prev = filt
```
`_temporal_filter(new, prev, alpha=0.5)`等价：
- `filt = 0.5*new + 0.5*prev`
作用：
- 降低帧间抖动，让输出更平滑。
- `self._prev` 保存上一帧滤波结果，实现“递推”。

4. 坐标变换/旋转（用于对齐安装方向）**这个操作应该是自己写的**
```
rotated_filt = np.flipud(np.rot90(filt, k=5))
```
拆开解释：
- np.rot90(filt, k=5)：旋转 90° * 5 次
  90°×5 = 450°，等价于 90°（因为 360° 一圈）
  所以它其实就是“旋转 90°”。
- 然后 np.flipud(...)：上下翻转。

5. 打包ros 的信息然后发布
   ```
    msg = TactileInput()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.local_time = str(time.time())
    msg.data = rotated_filt[:12].flatten().astype(float).tolist()

    self._pub.publish(msg)
    self._last_pub = time.time()
   ```
关键点：
- rotated_filt[:12]：只取前 12 行（也就是 12×16=192 个点）说明你的下游消息 TactileInput.data 可能只期望 192 维，而不是完整 256 维。
- .flatten()：展平成一维（按行展开）
- .tolist()：ROS msg 里一般需要 Python list



### 注意
**经过对传感器逻辑的学习，我发现了重点：这个传感器并不能反映出力的大小，而是只能反映出力的相对大小。
每次接触肯定是有一些地方没有和传感器接触，所以一定会有一些地方收集到的力为0。这个是没有问题的。
然后但是如果一次最大力是500N，一次最大力是20N，最后这个max值反映的结果肯定都是1，无法真实地反映出力本身的大小，只能反映出一帧的相对力，可以将一帧的受力情况比较好的展现出来，或者说可以把与传感器接触的物体压力分布比较好的勾勒出来，更适合做接触位置、形状，但是无法反映绝对力大小。**
**关于这里触觉传感器的力的处理方式，我在一次给老师的汇报中有明确的说明，他的整体设计是没有问题的，是一种比较好的可以反映出接触触觉情况的方案。**

---
## 