# 未知型号 Go2 的实物适配契约

## SDK 运动端

`UnitreeMotion` 仅调用官方 `SportClient.Move(vx, vy, wz)` 和 `StopMove()`。
不直接写关节、不自动起立、不关闭运动服务。DDS 通信不要求 ROS 2。
官方 SDK 是否开放给具体设备，必须以型号、固件、权限及实际返回码为准。

安装参考：<https://github.com/unitreerobotics/unitree_sdk2_python>。
硬件型号未知，因此没有自动安装 SDK 或探测机器人网络。

## 雷达插件

`--lidar-provider package.module:factory` 加载 `factory(config)`，返回可迭代对象。
每一帧为 `go2_rescue.unitree_sensors.PointCloudPacket`：

```python
PointCloudPacket(
    timestamp=acquisition_host_monotonic_seconds,
    points=xyz_in_lidar_frame,   # numpy Nx3, metres
    point_times=None,           # 可选，长度N的点时间
)
```

原生适配器用 `body_from_lidar` 变换点云。插件负责实际雷达通信、二进制解码、单位换算、去除机器人自遮挡，以及将设备时钟映射到本机单调时钟。
禁止将未知布局下的全局地图、真值坐标、裁判标签放进观测。
插件实现 `close()` 时会在退出时调用。

原生 SDK 层订阅 `rt/sportmodestate`，提取 IMU wxyz 四元数、速度和 yaw_speed；相机通过 VideoClient 获取。
若该型号接口不同，应修改适配器，不修改算法层来猜测含义。

## 本机桥接协议

适用于已有雷达/相机采集程序或 ROS 1 桥。核心算法仍不依赖 ROS。
传感器进程监听 `127.0.0.1:8765`，程序作为 TCP 客户端接收：

`4字节大端无符号JSON长度 + UTF-8 JSON`

单包最多16 MiB，源应按10 Hz左右发送同步帧；不累计历史包，出现背压应丢弃旧帧。

```json
{
  "schema": "go2-rescue-frame/1",
  "point_frame": "body_flu",
  "clock": "host_monotonic",
  "timestamp": 12345.60,
  "points": [[1.2, 0.1, -0.35], [2.0, 0.3, 0.1]],
  "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
  "velocity": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
  "state_age": 0.01,
  "image_age": 0.02,
  "image_jpeg": "可选的JPEG base64字符串"
}
```

`timestamp` 是雷达采集时刻对应的本机 `time.monotonic()`；速度为测得的机体速度，不是上次下发的命令。
`state_age` 和 `image_age` 是数据相对于雷达时刻的保守年龄/同步误差（秒，非负）。接收端还加上传输和处理耗时。
`image_jpeg` 缺失时允许导航，但不能据此确认新的球或终点。`point_times` 可选；当前核心没有对点云做逐点去畸变，不应伪造时间。

## 标定约定

- 机体：x前、y左、z上，单位米；角速度弧度/秒。
- 相机：OpenCV光学坐标，x右、y下、z前。
- `body_from_camera` 和 `body_from_lidar` 是右乘列向量的4×4刚体变换。
- `base_height` 是附近地面到机体原点的实测高度。
- `camera_size` 必须与真实帧一致，不能只缩放图像而不更新内参。
- 标定模板中的数值是软件场景参数，不是 Go2 出厂标定。

## 实机验收顺序

先录制静止/手持传感器数据，验证坐标、时延、地图和目标位置；再验证SDK停车及低速指令，最后运行自主任务。
原始录制不覆盖，回放比较目标漏报/误报、定位漂移、路径碰撞与状态事件。
模型未知阶段无法完成雷达驱动、外参和真实运动验收，这些是部署前必须补齐的设备信息。
