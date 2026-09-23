# Mars Lander · Phase 2

四腿地形自适应火星降落器原型（ENGR1000 Phase 2）。下视相机 + IMU 做地形研究与姿态估计，四路直线推杆独立调节着陆平台。

## 仓库结构

| 路径 | 内容 |
|---|---|
| `project/` | 主工程：ESP32-P4 固件、Mega 执行器控制、PC 控制台、传感器测试 |
| `project/finalware/` | 当前部署固件快照与控制台（优先看这里） |
| `project/main/` | P4 主机相机/HTTP 固件 |
| `project/wifi_test/` | Wi-Fi 图传 / IMU 时间戳链路测试 |
| `project/usb_cdc_mega_test/` | Arduino Mega 四腿 VL53 闭环 + PS2 遥控 |
| `project/pc_viewer/` | Windows 控制台 `lander_console.py`、深度可视化 |
| `project/mechanical/` | 控制盒 CAD 生成脚本与 STEP/STL |
| `CAD/` | 外购件/结构 STEP 参考模型 |
| `docs/` | 交接文档、系统说明、接线总表、答辩 PPT、LaTeX |
| `experiments/` | MoGe 录屏深度实验 |
| `media/videos/` | 验证视频与控制台录屏（**不进 Git**，见 `.gitignore`） |

## 快速开始

### 固件（ESP-IDF v5.4+ / v5.5）

```bash
cd project          # 或 finalware/wifi_test 等子工程
idf.py set-target esp32p4
idf.py build
idf.py -p COMx flash monitor
```

### PC 控制台

```bash
cd project/pc_viewer
pip install -r requirements.txt
python lander_console.py
# 或 start_lander_console.bat
```

### Mega 执行器

Arduino IDE 打开 `project/usb_cdc_mega_test/faithful_remote_hybrid/faithful_remote_hybrid.ino` 烧录 Mega 2560。

## 安全边界（Stage 1）

- **Remote / CV Control**：遥控与 16 组预设，**不是**自动 CV 驱动腿。
- **IMU Control**：BNO085 姿态闭环，与 Remote 互斥。
- **相机 / 深度 / SR04 不得进入 Stage 1 腿指令路径**。
- `CV_CONTROL_VALIDATED = False` 仍为强制约束。

硬件映射、当前部署状态与调试步骤见 [`docs/HANDOVER.md`](docs/HANDOVER.md) 与 [`docs/PROJECT_SYSTEM_EXPLANATION.md`](docs/PROJECT_SYSTEM_EXPLANATION.md)。接线见 [`docs/火星降落器接线总表.md`](docs/火星降落器接线总表.md)。

## 硬件一览

| 层级 | 器件 | 职责 |
|---|---|---|
| 高层网关 | ESP32-P4 Function EV | 相机、BNO085、Wi-Fi/HTTP、USB Host、转发 Mega 指令 |
| 执行器 | Arduino Mega 2560 | 四路 VL53 位置闭环、PS2、电机、STOP/ESTOP |
| 传感器扩展 | TCA9548A | 四路同地址 VL53L1X |
| 惯性 | BNO085 | 姿态 |
| 相机 | OV5647 | 800×800 下视鱼眼 |
| 测距锚点 | HC-SR04 | 仅深度尺度研究输入 |

## 本地未入仓内容

验证视频、控制台录屏在 `media/videos/`；`build/`、`managed_components/`、Python venv、模型权重、Waveshare 示例包等可再生成，已由 `.gitignore` 排除。重新构建时 `idf.py` 会拉取 managed components。
