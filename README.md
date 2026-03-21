# Pico_grblHAL_for_PD42S1

基于 grblHAL 的 RP2040/Pico2 W CNC 控制器固件 - PD42S1 板级支持

## 📋 项目概述

本项目是为 **PD42S1** 开发板定制的 grblHAL 固件，基于 Raspberry Pi Pico2 W（RP2350）微控制器。

### 主要特性

- ✅ **4 轴运动控制** (XYZA)
- ✅ **龙门架双 Y 电机** (A 轴与 Y 轴联动)
- ✅ **250MHz 超频** - 充分利用 RP2350 性能
- ✅ **蓝牙 SPP over GATT** - 无线串口通信
- ✅ **USB 串口** - 标准 CDC 接口
- ✅ **激光 PWM 控制** - GP1 引脚输出
- ✅ **高速运动** - 最大速度 30000 mm/min

---

## 🎯 硬件规格

### 核心板
- **芯片**: Raspberry Pi Pico2 W (RP2350)
- **主频**: 250MHz (超频)
- **无线**: CYW43439 (蓝牙 5.0 + WiFi)
- **闪存**: 16MB QSPI Flash

### 引脚配置

| 功能 | X 轴 | Y 轴 | Z 轴 | A 轴 |
|------|------|------|------|------|
| **步进脉冲** | GP19 | GP8 | GP12 | GP4 |
| **方向控制** | GP20 | GP9 | GP11 | GP3 |
| **使能信号** | GP18 | GP7 | GP13 | GP5 |
| **限位开关** | GP21 | GP14 | GP10 | GP2 |

### 其他引脚
- **激光 PWM**: GP1
- **I2C**: 已禁用（释放 GP26/27）

---

## 🔧 配置说明

### 1. 龙门架双 Y 电机配置

本固件专为龙门架结构设计，A 轴与 Y 轴联动：

```gcode
; 启用 Y_GANGED 模式
; A 轴作为 Y 轴的从轴，同步运动
; $3=8 设置 A 轴方向反转，确保双电机同步
```

**配置文件**: [`pd42s1_config.gcode`](pd42s1_config.gcode)

包含参数：
- `$110-$113 = 30000` - 各轴最大速度 (mm/min)
- `$120-$123` - 各轴加速度 (mm/sec²)
- `$3 = 8` - A 轴方向反转

### 2. 测试程序

提供完整的测试 G-code 程序：[`pd42s1_test.gcode`](pd42s1_test.gcode)

测试内容：
- 慢速同步性测试 (200 mm/min)
- 中速同步性测试 (1000 mm/min)
- 高速性能测试 (5000 mm/min)
- 矩形路径综合测试
- 对角线联动测试
- 圆弧插补测试

---

## 📥 安装与使用

### 编译固件

```bash
cd build
cmake .. -G Ninja -DPICO_BOARD=pico2_w
cmake --build . --config Release
```

生成的固件：`build/grblHAL.uf2`

### 烧录固件

1. 按住 Pico2 W 上的 **BOOTSEL** 按钮
2. 连接 USB 到电脑
3. 松开 BOOTSEL 按钮
4. 将 `grblHAL.uf2` 复制到出现的 **RPI-RP2** 驱动器

### 应用配置

#### 方法一：串口工具
1. 打开 PuTTY、Tera Term 或 Arduino IDE 串口监视器
2. 连接参数：
   - 波特率：115200
   - 数据位：8
   - 停止位：1
   - 校验：None
3. 复制粘贴 `pd42s1_config.gcode` 的内容
4. 发送后重启机器

#### 方法二：CNC 软件
1. 使用 CNCjs 或 Universal Gcode Sender
2. 加载 `pd42s1_config.gcode`
3. 点击"运行"执行配置

### 验证配置

```gcode
$$          ; 查看所有参数
$110        ; 检查 X 轴最大速度
$111        ; 检查 Y 轴最大速度
$113        ; 检查 A 轴最大速度
$3          ; 检查方向设置（应为 8）
```

### 运行测试

```gcode
; 加载并运行 pd42s1_test.gcode
; 观察 Y 轴和 A 轴是否同步运动
```

---

## 🏗️ 项目结构

```
Pico_grblHAL_for_PD42S1/
├── boards/
│   └── pico_pd42s1_map.h      # PD42S1板级配置文件
├── grbl/                       # grblHAL核心代码
├── my_machine.h                # 机器配置（Y_GANGED 在此启用）
├── main.c                      # 主程序（250MHz 超频）
├── CMakeLists.txt              # CMake构建配置
├── pd42s1_config.gcode         # 配置脚本
├── pd42s1_test.gcode           # 测试程序
└── build/                      # 编译输出目录
    └── grblHAL.uf2             # 可烧录固件
```

---

## ⚙️ 技术参数

### 运动控制
- **最大速度**: 30000 mm/min (所有轴)
- **加速度**: 
  - X/Y/A轴：50 mm/sec²
  - Z 轴：20 mm/sec²
- **步距**: 需在配置时设置 (`$100-$103`)

### 通信接口
- **USB 串口**: 115200 波特率
- **蓝牙 SPP**: 支持无线连接
- **WiFi**: 可选（需额外配置）

### 特殊功能
- **激光控制**: M3/M5 命令，GP1 PWM 输出
- **限位保护**: XYZA 四轴硬限位
- **急停控制**: 可通过软件实现

---

## 🔍 故障排查

### A 轴不跟随 Y 轴
1. 检查 `my_machine.h` 中 `#define Y_GANGED 1` 是否启用
2. 确认固件已正确烧录
3. 检查 `$3` 参数是否为 8

### 双 Y 轴不同步
1. 调整机械传动比
2. 检查皮带/丝杆张紧力
3. 降低加速度 (`$121/$123`)
4. 增加电机驱动电流

### 高速振动
1. 降低最大速度 (`$110-$113`)
2. 降低加速度 (`$120-$123`)
3. 检查机械刚性

---

## 📄 许可证

本项目基于 grblHAL，遵循 GPL v3 许可证。

---

## 🙏 致谢

### 原固件仓库
- **grblHAL/RP2040**: https://github.com/grblHAL/RP2040
  - 官方 RP2040 平台驱动
  - 提供了完整的基础框架

- **grblHAL/grblHAL**: https://github.com/grblHAL/grblHAL
  - 核心 grblHAL 项目
  - 高级 CNC 控制功能

### 社区资源
- **Raspberry Pi Pico SDK**: https://github.com/raspberrypi/pico-sdk
- **grblHAL 文档**: http://www.grbl.org/

---

## 📞 支持与反馈

### 问题反馈
如有问题或建议，请通过以下方式联系：

- **GitHub Issues**: https://github.com/kxgx/Pico_grblHAL_for_PD42S1/issues
- **grblHAL 论坛**: https://www.grbl.org/forums

### 相关文档
- [grblHAL 官方文档](http://www.grbl.org/)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [G-code 标准](https://linuxcnc.org/docs/html/gcode/gcode.html)

---

## 📝 更新日志

### v1.0 - 2026.03.22
- ✅ 初始版本
- ✅ PD42S1板级支持
- ✅ 4 轴系统 (XYZA)
- ✅ A 轴与 Y 轴联动（龙门架模式）
- ✅ 250MHz 超频
- ✅ 蓝牙 SPP 支持
- ✅ 高速运动配置 (30000 mm/min)
- ✅ A 轴方向反转 (`$3=8`)
- ✅ 完整测试程序

---

## 🔗 相关链接

- **项目主页**: https://github.com/kxgx/Pico_grblHAL_for_PD42S1
- **grblHAL 官网**: http://www.grbl.org/
- **Raspberry Pi Pico**: https://www.raspberrypi.com/products/raspberry-pi-pico/

---

**祝使用愉快！** 🚀
