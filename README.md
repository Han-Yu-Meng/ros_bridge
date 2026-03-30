# ROS Bridge 库

封装 ROS 2 I/O 节点，统一为 FineVision DAG 提供发布、订阅、TF 广播三类功能。所有节点均基于模板，可与任意 ROS 消息配合使用。

## 节点

| 节点 | 类型 | 说明 | 输入/输出 | Extern |
| --- | --- | --- | --- | --- |
| `ROSPub<MsgT>` | `Executor` | 将内部消息发布到 ROS 主题 | 输入：`MsgT`；输出：- | `ros_topic` |
| `ROSSub<MsgT>` | `Executor` | 订阅 ROS 主题并推送到 DAG，支持按需使用 ROS 时间戳。 | 输出：`MsgT` | `ros_topic`、`use_ros_time` |
| `TFBroadcaster` | `Executor` | 将 `geometry_msgs::msg::TransformStamped` 广播到 TF2，强制设置 frame_id 和 child_frame_id。 | 输入：Transform | `from_frame`、`to_frame` |
| `TransformRPY` | `Executor` | 生成 `geometry_msgs::msg::TransformStamped` 从 RPY 参数，50Hz | 输出：Transform | `tx`, `ty`, `tz`, `roll`, `pitch`, `yaw` |
| `TransformWXYZ` | `Executor` | 生成 `geometry_msgs::msg::TransformStamped` 从四元数参数，50Hz | 输出：Transform | `tx`, `ty`, `tz`, `rx`, `ry`, `rz`, `rw` |
| `LookupTransform` | `Executor` | 查询 TF 变换从 from_frame 到 to_frame，50Hz | 输出：Transform | `from_frame`, `to_frame`, `duration_ms` |

### 使用方式

```cpp
auto imu_sub = Executor<ROSSub<sensor_msgs::msg::Imu>>("IMUSubscriber");
auto cloud_pub = Executor<ROSPub<sensor_msgs::msg::PointCloud2>>("CloudPublisher");
auto tf_broadcaster = Executor<TFBroadcaster>("TFBroadcaster");
auto transform_rpy = Executor<TransformRPY>("TransformRPY");
auto transform_wxyz = Executor<TransformWXYZ>("TransformWXYZ");
auto lookup_transform = Executor<LookupTransform>("LookupTransform");
```

- 订阅/发布节点都会在 `initialize()` 时创建独立 ROS 节点与线程，不阻塞工作流主循环。
- `ROSPub`/`ROSSub` 通过 extern 切换话题或是否使用 ROS Header 时间。
