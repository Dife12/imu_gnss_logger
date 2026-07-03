# imu-gnss-logger

## 中文

### 当前程序

`imu-gnss-logger` 是一个基于 ESP32-S3 的 IMU + GNSS 数据记录程序，用于稳定采集传感器数据，并将结果保存到 SD 卡 CSV 文件。

当前实现只专注于以下功能：

- 采集 BMI270 IMU 数据
- 读取 u-blox GNSS 数据
- 可选使用 PCF8563 RTC 作为时间回退
- 以 IMU 采样为主，给每条 IMU 数据附上最近一次 GNSS fix
- 将数据持续写入 SD 卡

### 硬件

- 主控：Seeed Studio XIAO ESP32-S3 或兼容 ESP32-S3
- IMU：BMI270
- GNSS：u-blox NEO-F10N 或兼容 UART 模块
- RTC：PCF8563
- 存储：SPI SD 卡

### 当前引脚配置

- `SD_CS_PIN = D2`
- `GNSS_RX_PIN = D0`
- `GNSS_TX_PIN = D1`
- I2C：默认使用开发板 `Wire.begin()` 默认引脚

配置集中在 [include/imu_gnss_logger_config.h](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger/include/imu_gnss_logger_config.h)。

### 程序结构

- [src/main.cpp](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger/src/main.cpp)：主程序与 FreeRTOS 任务
- [src/imu_gnss_logger_csv.cpp](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger/src/imu_gnss_logger_csv.cpp)：CSV 格式化
- [include/imu_gnss_logger_types.h](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger/include/imu_gnss_logger_types.h)：数据结构
- [platformio.ini](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger/platformio.ini)：PlatformIO 配置

当前任务划分：

- `imuTask`：按 200Hz 采集 IMU
- `gnssTask`：按 20Hz 刷新 GNSS 状态
- `loggerTask`：从队列读取记录并写入 SD 卡

### CSV 格式

表头固定为：

```text
timestamp_ms,datetime_utc,imu_sample_id,gnss_fix_id,lat,lon,alt_m,speed_mps,heading_deg,gnss_fix_quality,gnss_satellites,ax,ay,az,gx,gy,gz,imu_dt_ms,gnss_age_ms
```

说明：

- `timestamp_ms`：`millis()` 单调时间
- `datetime_utc`：优先使用 GNSS UTC，无则回退 RTC，无则 `NA`
- `imu_sample_id`：每条 IMU 记录递增
- `gnss_fix_id`：每个新的有效 GNSS fix 递增
- `gnss_age_ms`：当前 IMU 样本使用的 GNSS 数据距离最新更新时间的延迟
- 缺失值统一写 `NA`

示例见 [data/sample/sample_log.csv](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger/data/sample/sample_log.csv)。

### 编译与烧录

本项目是标准 PlatformIO 工程。

- 环境：`seeed_xiao_esp32s3`
- 框架：Arduino

在 VS Code 中：

1. 打开 [imu_gnss_logger](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger)
2. 安装 PlatformIO IDE 扩展
3. 运行 `Build`
4. 连接开发板后运行 `Upload`
5. 运行 `Monitor`

命令行：

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

### 运行结果

启动后程序会：

1. 初始化串口
2. 初始化 SD 卡并创建新的 `LOG_XXXX.CSV`
3. 初始化 IMU
4. 初始化 GNSS
5. 如果存在 RTC，则初始化 RTC
6. 启动采集与写卡任务

### 当前限制

- 当前环境没有安装 `pio`，所以这里没有做本地实际编译验证
- `gnss_fix_quality` 当前直接记录 u-blox `fixType`
- `alt_m`、`speed_mps`、`heading_deg` 依赖 GNSS 模块当前输出
- I2C SDA/SCL 没有在仓库中显式定义，默认使用板级引脚

## 日本語

### 現在のプログラム

このプロジェクトは、ESP32-S3 上で BMI270 IMU と u-blox GNSS を取得し、SD カードへ CSV として保存するロガーです。

現在の実装は次の機能に絞っています。

- BMI270 の取得
- GNSS の取得
- RTC を使った時刻フォールバック
- IMU サンプルごとに最新 GNSS fix を付与
- SD カードへの継続記録

### 構成

- ボード：XIAO ESP32-S3
- IMU：BMI270
- GNSS：u-blox UART モジュール
- RTC：PCF8563
- SD：SPI SD カード

### ビルド

- PlatformIO プロジェクト
- 環境：`seeed_xiao_esp32s3`

VS Code で [imu_gnss_logger](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger) を開き、PlatformIO から `Build` / `Upload` / `Monitor` を実行してください。

### 出力

各 IMU サンプルにつき 1 行の CSV を出力します。CSV サンプルは [data/sample/sample_log.csv](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger/data/sample/sample_log.csv) を参照してください。

## English

### Current Program

This project is an ESP32-S3 IMU + GNSS logger that reads BMI270 and u-blox sensor data and writes synchronized CSV records to an SD card.

The current code focuses only on:

- BMI270 sampling
- GNSS polling
- Optional RTC fallback
- One CSV row per IMU sample
- Attaching the latest GNSS fix to each IMU row
- Continuous SD card logging

### Build

- PlatformIO project
- Environment: `seeed_xiao_esp32s3`
- Framework: Arduino

Open [imu_gnss_logger](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger) in VS Code and use PlatformIO to `Build`, `Upload`, and `Monitor`.

### Output

The logger writes `LOG_XXXX.CSV` files to the SD card. Each row contains one IMU sample plus the latest available GNSS fix. A sample file is available at [data/sample/sample_log.csv](/Users/tekihi/Documents/RoadSense Toolkit/imu_gnss_logger/data/sample/sample_log.csv).

### Notes

- Local PlatformIO build was not executed in this environment
- `gnss_fix_quality` currently stores u-blox `fixType`
- Board-default I2C pins are used unless you change the config
