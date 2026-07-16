# 2023 电赛 E 题 MaixCAM Pro 视觉程序

本仓库只包含可直接在 MaixVision 中运行的两个独立视觉项目：

- `maixvision_projects/red_target`：红色运动目标系统。识别 A4 靶纸轨迹，生成 20 个顺时针航点，并输出航点和视觉误差帧。
- `maixvision_projects/green_tracker`：绿色自动追踪系统。识别红、绿光斑并输出 `red - green` 视觉误差。

两个项目分别部署到各自的 MaixCAM Pro 与控制系统，比赛运行时不相互通信。

## 在 MaixVision 中运行

1. 选择“文件 → 打开文件夹/项目”。
2. 红色系统打开 `maixvision_projects/red_target`；绿色系统打开 `maixvision_projects/green_tracker`。
3. 点击“运行项目”，不要点击“运行当前文件”。运行项目会一起发送 `main.py` 和依赖模块。

红色项目打开正确时，第一层应看到：

```text
main.py
config.py
rectangle_lock.py
touch_controls.py
tracker_state.py
trajectory.py
shared/
```

红色系统使用 `640×480` 作为主画面、航点和 UART 坐标；为适配 MaixCAM Pro 的快速帧缓冲，矩形检测在 `320×240` 临时图像上运行，再将角点映射回主坐标。

## 现场标定

运行前请根据现场环境调整各项目 `config.py` 中的 LAB 阈值、曝光、UART 参数、到达半径和矩形筛选参数。首次联调时应先断开电机或限制云台输出，确认误差符号和丢失保护后再接入 PID。

## UART 帧

固定 9 字节帧：

```text
AA 55 TYPE INDEX DATA0L DATA0H DATA1L DATA1H CHECKSUM
```

- `TYPE=01`：当前航点，每 100 ms 重发。
- `TYPE=02`：视觉误差，供 STM32 PID 使用。
- `TYPE=03`：红色轨迹完成。
- `TYPE=04`：光斑丢失。
- `TYPE=05`：绿色追踪状态。
