# Radar Python 上位机

该目录是 STM32 雷达项目配套的 Python 上位机应用，满足以下功能：

- 实时接收串口数据并绘制心率/呼吸曲线
- 心率与呼吸可独立开始/停止测量
- 自动检测串口设备
- 结束测量后计算平均值并本地持久化保存
- 支持按时间范围查询历史记录

## 目录结构

- `main.py`：程序入口
- `app/main_window.py`：主界面与交互逻辑
- `app/serial_client.py`：串口通信与数据解析
- `app/storage.py`：SQLite 持久化
- `app/models.py`：数据模型
- `data/measurement_history.db`：运行后自动创建

## 运行环境

- Python 3.10+
- Windows / Linux / macOS

## 安装依赖

```bash
pip install -r requirements.txt
```

## 启动

```bash
python main.py
```

## 串口调试

如果界面看不到曲线，可先运行串口抓包工具：

```bash
python tools/serial_probe.py --port COM9 --baud 115200 --seconds 10
```

说明：

- `RX=0`：说明没有收到任何串口数据，优先检查线序、波特率、串口号、下位机是否启动。
- `RX>0 但解析成功=0`：说明收到数据但格式不匹配，请把输出贴出来再做协议适配。
- `解析成功>0`：说明链路正常，界面问题通常在绘图启停逻辑。

## 下位机串口协议（UART1）

当前版本采用“下位机固定周期主动上报”模式，上位机默认不发送控制命令。

### 下位机遥测数据

```text
RADAR,tick_ms,heart_rate,breath_rate,heart_phase,breath_phase,distance_cm,human_present,online,range_valid,heart_enabled,breath_enabled
```

字段均为 ASCII CSV，换行结尾。

## 历史记录

结束一次测量（点击停止按钮）后，会写入一条记录到 SQLite：

- 类型（HEART/BREATH）
- 开始时间
- 结束时间
- 平均值
- 样本数
- 串口名
