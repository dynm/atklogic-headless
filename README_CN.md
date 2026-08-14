# atklogic-headless

[English](README.md) | [简体中文](README_CN.md)

面向 ALIENTEK ATK-Logic DL16 系列逻辑分析仪的非官方无 GUI 采集工具，同时提供可直接安装的 Codex Skill。

Unofficial headless capture client and Codex Skill for the ALIENTEK ATK-Logic DL16 family.

## 功能

- 通过 PyUSB 探测和操作 `1a86:ffcc` 设备
- 支持立即采集以及低电平、高电平、上升沿、下降沿和双沿触发
- 支持 16 个数字通道和常用采样率
- 输出逐通道 packed binary、JSON 元数据、VCD 波形及原始 USB 数据
- 内置 10 Mbps FlexRay 帧解码、CRC 检查和波形时序报告
- 提供 pico-flexray slot10 `with11` / `without11` 测试辅助功能

目前只安全导出 Stream 模式。Buffer 模式由于环形缓冲区偏移尚未完整处理，会被程序主动拒绝。

## 已验证硬件

2026-08-14 在 ATK-Logic DL16 系列设备上完成：

- MCU 探测与版本查询
- 100 MHz、双通道、10 ms 立即采集
- 100 MHz、单通道、2 ms 低电平触发采集
- JSON、VCD、packed channel 和 USB raw 输出一致性检查

不同硬件和固件版本仍可能存在差异，欢迎提交 issue 并附上命令、错误输出和设备版本。

## 安装

需要 Python 3、PyUSB 和系统 libusb。

```bash
git clone https://github.com/dynm/atklogic-headless.git
cd atklogic-headless
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

macOS 可通过 Homebrew 安装 libusb：

```bash
brew install libusb
```

Debian/Ubuntu 可安装运行库：

```bash
sudo apt install libusb-1.0-0
```

安装为个人 Codex Skill：

```bash
git clone https://github.com/dynm/atklogic-headless.git ~/.codex/skills/atklogic-headless
```

## 使用

先关闭可能占用 USB 接口的 ATK-Logic GUI，然后探测设备：

```bash
python3 scripts/atk_logic_headless.py probe
```

立即采集 USB CH0 和 CH1：

```bash
python3 scripts/atk_logic_headless.py capture \
  --output captures/smoke \
  --channels 0,1 \
  --rate 100M \
  --duration-ms 10 \
  --threshold 1.6 \
  --instant
```

下降沿触发：

```bash
python3 scripts/atk_logic_headless.py capture \
  --output captures/triggered \
  --channels 0,1 \
  --rate 100M \
  --duration-ms 100 \
  --trigger-channel 0 \
  --trigger falling \
  --trigger-position 10
```

通道编号从零开始：USB `CH0` 对应面板 `CH1`。Stream 模式限制为：

- 1–3 个通道：最高 100 MHz
- 4–6 个通道：最高 50 MHz
- 7–16 个通道：最高 20 MHz

查看所有参数：

```bash
python3 scripts/atk_logic_headless.py capture --help
```

## 输出格式

对于输出前缀 `PREFIX`：

- `PREFIX.json`：采集配置、边沿摘要和解码结果
- `PREFIX.vcd`：可由 GTKWave 等工具打开的波形
- `PREFIX.chN.bin`：LSB-first packed samples
- `PREFIX.usb.bin`：用于故障定位的原始 USB 数据

Packed 数据中，样本 `N` 位于字节 `N // 8` 的 bit `N & 7`。

## FlexRay 解码

```bash
python3 scripts/atk_logic_headless.py capture \
  --output captures/flexray \
  --channels 0,4,5 \
  --rate 100M \
  --duration-ms 100 \
  --instant \
  --flexray-channel 4 \
  --flexray-channel-type A
```

slot10 相关参数是 pico-flexray 测试台辅助功能；通用采集和 FlexRay 解码不依赖该项目。

## 来源与许可证

USB 协议实现参考并移植自 ALIENTEK 官方公开仓库 [`alientek-openedv/atk-logic`](https://github.com/alientek-openedv/atk-logic)，基准提交：

```text
0dff562d24436def2bec3791684f1911997b9e35
```

主要公开参考文件包括：

- `pv/usb/usb_base.cpp`
- `pv/usb/usb_control.cpp`
- `pv/static/util.cpp`
- `pv/controller/session_controller.cpp`
- `pv/data/session.cpp`

面向不同设备版本的兼容差异通过自有硬件 A/B 试验确定。本仓库不包含厂商应用程序、固件、抓包样本或其他非公开资源。

上游项目采用 GPL-3.0-or-later；本项目作为 Python 移植和扩展，同样整体采用 [GPL-3.0-or-later](LICENSE)。修改日期为 2026-08-14。

这是非官方社区项目，与 ALIENTEK 没有隶属或背书关系。ATK-Logic 等名称及商标归其各自权利人所有。

## 安全提示

- 连接目标电路前确认分析仪和目标设备共地。
- 阈值必须适合目标逻辑电平。
- 原始 USB 数据和波形可能包含被测系统信息，公开 issue 前请先检查。
- 固件升级、PWM 和其他写入型设备管理功能不在本工具范围内。
