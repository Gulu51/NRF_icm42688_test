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
main                         已测试的稳定版本
feature/phase-aware-low-power 分阶段采样与低功耗开发
```

不要直接在 `main` 上进行未经验证的算法修改。新功能测试通过后再合并到 `main` 并创建新的版本标签。
