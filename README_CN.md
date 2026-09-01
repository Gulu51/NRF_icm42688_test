# NRF52832 + ICM42688P 膝关节传感器

这是基于 Nordic nRF5 SDK 16.0.0 `ble_app_uart` 示例开发的 NRF52832、ICM42688P SPI 采集与 BLE NUS 通信工程。

## 当前稳定版本

- ICM42688P 加速度计和陀螺仪采样率：200 Hz
- BLE 状态发送频率：1 Hz
- 支持手机发送 `START`、`STOP` 和 `CLOSE`
- `STOP`/断开连接后关闭 IMU，降低非采样阶段功耗
- 手机接收累计步数与振动状态

稳定版本使用 Git 标签 `v0.1-tested-icm42688` 保存。后续分阶段采样、FIFO 和 BLE 低功耗改动应在独立功能分支完成。

## 工程位置

Keil MDK 工程：

`pca10040/s132/arm5_no_packs/ble_app_uart_pca10040_s132.uvprojx`

目标芯片为 nRF52832，使用 S132 SoftDevice。请将本仓库目录放在 nRF5 SDK 16.0.0 的 `examples/ble_peripheral/ble_app_uart` 位置，保持 SDK 相对路径不变。

## 推荐开发流程

```text
main                                  已测试的稳定版本
feature/phase-aware-low-power         1 Hz试验机分阶段采样版本
feature/mobile-low-power-step-warning 当前非试验机低功耗计步版本
```

不要直接在 `main` 上进行未经验证的算法修改。新功能测试通过后再合并到 `main` 并创建新的版本标签。

## `feature/mobile-low-power-step-warning` 当前测试分支

这一分支不依赖1 Hz试验机相位，适合当前手持走动和功耗仪测试：

- `START`后加速度计持续工作在25 Hz低功耗模式。
- 陀螺仪和温度传感器始终关闭。
- MCU由40 ms RTC定时事件唤醒，每次SPI只读取6字节三轴加速度数据，处理结束立即睡眠。
- 计步采用三轴动态加速度模长和迟滞/320 ms防重复机制，不依赖固定安装方向。
- 常规步数与振动状态每5秒合并发送一次。
- 强冲击超过1.20 g动态峰值时立即发送警告；持续强振动还会通过1秒RMS窗口判定。
- `STOP`关闭IMU并保持连接；`CLOSE`关闭IMU、断开蓝牙并进入System OFF。

### 手机命令

| 命令 | 功能 |
| --- | --- |
| `START` | 启动25 Hz低功耗计步和强振动监测 |
| `RESET` | 步数清零并清除振动报警状态 |
| `STOP` | 停止监测、关闭IMU，但保持蓝牙连接 |
| `CLOSE` | 关闭IMU、断开蓝牙并进入System OFF；复位或重新上电才能再次广播 |

启动成功会返回：

```text
MONITOR,ON
```

正常状态每5秒发送一次：

```text
S:120,V:0
```

检测到强烈振动时立即发送：

```text
WARNING,V:1
```

`V:1`是工程阈值警告，不代表膝关节磨损的临床诊断。25 Hz低功耗采样适合计步和明显冲击，但不能替代后续试验机上的高采样率磨损振动频谱分析。

### BLE低功耗参数

- 广播间隔：2 秒，广播功率 -20 dBm。
- 连接间隔：500 ms，从机延迟 4，监督超时 6 秒。
- 建立连接后也将连接发射功率设置为 -20 dBm。
- 常规结果每5秒合并发送；强烈振动立即上报。

不同手机可能拒绝从机提出的连接参数。功耗测试时应同时检查手机最终接受的连接间隔和从机延迟。
