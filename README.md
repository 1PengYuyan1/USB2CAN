# USB2CAN — Linux 下基于 USB2CAN 的达妙（DM）电机多路高频控制 Demo

**简体中文** | [English](README_EN.md)

基于 **Linux C++ + 肥猫（DingLAB）USB2CAN SDK（`libusb_can.so`）** 的多路 CAN 实时控制示例程序。通过两个 USB2CAN 模块扩展 4 路 CAN 总线（每路挂载 3 个达妙电机），以最高 **1000 Hz** 控制频率同时驱动 12 个达妙电机，适用于四足机器人或多关节机械臂底层通信验证。

> 本工程源自 [SOULDE-Studio/USB2CAN-Demo-DM](https://github.com/SOULDE-Studio/USB2CAN-Demo-DM)，整理去除了冗余嵌套目录与编译产物。

## 目录结构

```text
USB2CAN/
├── README.md                  # 中文主文档（本文件）
├── README_EN.md               # English README
├── LICENSE                    # Apache-2.0 许可
├── CMakeLists.txt
├── can/
│   ├── libusb_can.so          # USB2CAN 动态链接库
│   ├── usb_can.h              # USB2CAN C API 头文件
│   ├── usb_can.rules          # Linux udev 规则（固定设备权限与绑定）
│   └── readme                 # udev 绑定简明说明
├── include/
│   └── Tangair_usb2can.h      # 达妙电机 MIT 协议参数、数据结构与收发类定义
└── src/
    ├── Tangair_usb2can.cpp    # 多路 CAN 收发线程、MIT 模式打包/解包实现
    └── can_node.cpp           # 主程序入口（SCHED_FIFO 实时调度）
```

## 支持的达妙电机型号

`include/Tangair_usb2can.h` 内置了常见达妙电机 MIT 参数范围（`P / V / T`），包含：
- `DM4310` / `DM4310_48V`
- `DM4340` / `DM4340_48V`
- `DM6006` / `DM8006` / `DM8009` / `DM3519`
- `DM10010` / `DM10010L` / `DMH3510` / `DMH6215` / `DMG6220`

## 前期准备

### 1. 安装 USB2CAN udev 规则

```bash
cd can
sudo cp usb_can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2. 配置达妙电机 ID

使用达妙上位机为每台电机配置 **CAN ID** 与 **Master ID**（避免同一总线上反馈帧 ID 冲突；请对照 `src/Tangair_usb2can.cpp` 中的 ID 映射修改或对齐）。

## 编译与运行

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

程序使用 `SCHED_FIFO` 实时调度并直接访问 USB 设备，建议使用 `sudo` 运行（若运行时找不到 `libusb_can.so`，请指定 `LD_LIBRARY_PATH`）：

```bash
sudo LD_LIBRARY_PATH=../can ./can_node
```

> **安全提示**：首次上电测试前请确保电机空载且已设置好 `max_position` / `min_position` 限位及急停开关。

## 致谢与引用

原始 Demo 作者为 **TANGAIR / SOULDE-Studio**（Apache-2.0 License）：

```bibtex
@software{tangair2025USB2CAN-Demo-DM,
  author = {tangair},
  title = {{USB2CAN-Demo-DM: A project based on USB2CAN and DM motor.}},
  url = {https://github.com/SOULDE-Studio/USB2CAN-Demo-DM.git},
  year = {2025}
}
```
