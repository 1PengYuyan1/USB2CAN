# USB2CAN — Linux High-Frequency Damiao (DM) Motor Control Demo

[简体中文](README.md) | **English**

A Linux C++ real-time CAN control demo using the **DingLAB / Feimao USB2CAN SDK (`libusb_can.so`)**. Two USB2CAN adapters expand a Linux host to 4 CAN buses (3 Damiao motors per bus), controlling up to 12 Damiao motors at **1000 Hz** via MIT mode—suitable as a low-level actuator communication testbed for quadruped robots or multi-DOF manipulators.

> Based on [SOULDE-Studio/USB2CAN-Demo-DM](https://github.com/SOULDE-Studio/USB2CAN-Demo-DM), cleaned up by flattening the directory layout and removing committed build artifacts.

## Directory Layout

```text
USB2CAN/
├── README.md                  # Chinese primary README
├── README_EN.md               # English README (this file)
├── LICENSE                    # Apache-2.0 License
├── CMakeLists.txt
├── can/
│   ├── libusb_can.so          # Prebuilt USB2CAN shared library
│   ├── usb_can.h              # USB2CAN C API header
│   ├── usb_can.rules          # Linux udev rules
│   └── readme                 # Short udev setup note
├── include/
│   └── Tangair_usb2can.h      # DM motor MIT parameters, structs & manager class
└── src/
    ├── Tangair_usb2can.cpp    # Multi-bus TX/RX threads & MIT protocol encode/decode
    └── can_node.cpp           # Entry point (SCHED_FIFO real-time scheduling)
```

## Supported Damiao Motor Types

`include/Tangair_usb2can.h` defines MIT parameter ranges (`P / V / T`) for common Damiao actuators including `DM4310`, `DM4340`, `DM6006`, `DM8006`, `DM8009`, `DM3519`, `DM10010`/`DM10010L`, `DMH3510`, `DMH6215`, and `DMG6220`.

## Prerequisites

1. **Install udev rules**:

```bash
cd can
sudo cp usb_can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

2. **Configure motor CAN IDs & Master IDs** via the Damiao configuration tool so feedback frames do not collide on the same bus (matching `src/Tangair_usb2can.cpp`).

## Build & Run

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)

sudo LD_LIBRARY_PATH=../can ./can_node
```

> **Safety Note**: Always test motors unloaded with soft position limits (`max_position` / `min_position`) and an accessible emergency stop.

## Attribution

Original code Copyright (c) 2023–2025 **TANGAIR / SOULDE-Studio** (Apache-2.0 License):

```bibtex
@software{tangair2025USB2CAN-Demo-DM,
  author = {tangair},
  title = {{USB2CAN-Demo-DM: A project based on USB2CAN and DM motor.}},
  url = {https://github.com/SOULDE-Studio/USB2CAN-Demo-DM.git},
  year = {2025}
}
```
