All backup files are in the “release”
  # Kibo-RPC 7th Simulator

This workspace is used to develop programs for Kibo-RPC 7th and test them with the Web simulator or the local simulator.

For environment setup, API specifications, and detailed programming instructions, see the [Programming Manual](docs/ProgrammingManual.md).

## Before You Start

- Work in Linux or Ubuntu on WSL2.
- Docker is required when using the local simulator.
- The main directory for participant code is `users/catkin_ws/src/krpc_user/`.
- Do not modify provided files such as `docker/`, `rootfs/`, `rviz/`, `krpc_api_server`, or `krpc_ib2_msgs` unless necessary.

## Basic Usage

### 1. Edit Your Program

Edit the sample program in `users/catkin_ws/src/krpc_user/`.
Do not change the package name, `CMakeLists.txt`, `package.xml`, or launch file names.

### 2. Create a Submission Package

From the project root directory, run:

```bash
cd users
bash ./packing.sh my_submission
```

This creates `users/my_submission.tar.gz`. Upload this file to the Web simulator to run your program.

The Web simulator is available at:

- [Kibo-RPC Login Page](https://jaxa.krpc.jp/user-auth)

### 3. Use the Local Simulator

Build the Docker image the first time you use the local simulator or when the image is updated.

```bash
bash ./build.sh
```

Then, start the simulator in another terminal:

```bash
bash ./simulation.sh
```

While the simulator is running, open <http://localhost> in a browser to view it.
Your program is built and started automatically by `simulation.sh`.

To stop the simulation, press `Ctrl+C` in the terminal where it is running.

### 4. Check Results and Logs

Execution results are saved in `share/logs/`. The latest results are available in `share/logs/latest/`.

#### 備份資料夾內容（繁體中文）

每次模擬結束時，模擬器會將當次執行產生的結果存入以 `YYYYMMDDHHMMSS` 命名的資料夾。以下是備份內容及用途：

```text
share/logs/
├── <YYYYMMDDHHMMSS>/              # 單次執行的完整結果
│   ├── cheat_sheet.json
│   ├── iss.urdf
│   ├── simulation.bag
│   ├── textures/                   # ARmarker1.png 至 ARmarker12.png
│   ├── flight_recorder_logs/
│   ├── judge_module_logs/
│   └── user_programs_logs/
└── latest/                         # 最近一次執行結果的便利副本
```

**備份資料夾根目錄**

| 檔案／資料夾 | 內容 |
|---|---|
| `cheat_sheet.json` | 當次隨機任務環境的檢查點配置，例如檢查點位置、姿態和評分條件。用來重現或解讀該次任務，不是參賽程式碼。 |
| `simulation.bag` | ROS rosbag，記錄模擬期間的大部分 ROS topics，可用 `bash ./replay.sh` 回放。錄製時排除了名稱含 `airflow` 的 topic。 |
| `iss.urdf` | 當次回放所需的 ISS／模擬場景機器人描述檔。 |
| `textures/ARmarker*.png` | 當次場景使用的 AR marker 紋理圖；`.gitkeep` 是用來保留空資料夾的空白佔位檔。 |

**`judge_module_logs/`：裁判與模擬流程紀錄**

| 檔案 | 內容 |
|---|---|
| `result.json` | 裁判結果摘要，包含任務是否開始／完成、耗時、各檢查點通過狀態與等級、目標點結果及違規狀態。 |
| `data.log` | 裁判模組收到的原始任務事件資料，每行一筆，方便追查事件內容。 |
| `localization_history.txt` | 任務開始後定期記錄 Int-Ball 2 的位置與方向姿態；每行為一筆 JSON。 |
| `game.log` | 模擬啟動、狀態切換、等待與結束等主要流程訊息。 |
| `judge.log` | 裁判模組的輸出與錯誤訊息。 |
| `system.log` | Gazebo、ROS launch、環境產生和其他模擬器系統指令的輸出。 |
| `rosbag.log` | rosbag 錄製程序的狀態及錯誤訊息；實際錄製資料在根目錄的 `simulation.bag`。 |
| `status_indicator.log` | 狀態指示器模組的輸出與錯誤訊息。 |

**`flight_recorder_logs/`：任務軌跡紀錄**

| 檔案 | 內容 |
|---|---|
| `data.json` | 飛行紀錄 JSON，包含任務場景階段標記與 Int-Ball 2 的位置、方向軌跡資料。 |
| `ib2.log` | 以時間及 x/y/z 座標記錄 Int-Ball 2 的導航位置。 |
| `flight_recorder.log` | 飛行紀錄模組的執行狀態與錯誤訊息。 |

**`user_programs_logs/`：使用者程式的 ROS 輸出**

| 檔案／資料夾 | 內容 |
|---|---|
| `user_program_output.log` | 從 ROS console 擷取的使用者節點 `/krpc_user` 輸出，包含 ROS 訊息以及程式的標準輸出。 |
| `ros/krpc_user_console.log` | roslaunch 顯示的完整使用者程式 console 輸出，可用來對照擷取後的 `user_program_output.log`。 |
| `ros/<執行識別碼>/` | 該次 ROS launch 的節點記錄，例如 `krpc_user-*.log`、`krpc_api_node-*.log` 和 `roslaunch-*.log`；`ros/latest` 是 ROS 建立的最近執行記錄捷徑。 |

`share/logs/latest/` 是最近一次結果資料夾的副本，模擬器完成備份時會更新它；其中的 `link_origin` 連結指向對應的時間戳記資料夾。若另有 `<YYYYMMDDHHMMSS>.rar`，它是備份資料夾的壓縮檔；`save_logs` 流程本身不會建立 RAR 檔。

> **原始碼不包含在任務備份中。** `save_logs` 只保存上述任務資料、紀錄和場景資產，不會複製當次建置的使用者程式原始碼。原始碼仍在 `users/catkin_ws/src/krpc_user/`；若要保存每次執行時使用的程式版本，請另行以 Git commit/tag 或自行封存原始碼。`share/` 也列在 `.gitignore` 中，因此這些模擬結果不會隨一般 Git commit 上傳。

## Replay

To replay a rosbag created by a simulation, run:

```bash
bash ./replay.sh
```

You can specify the playback speed and start time as arguments:

```bash
bash ./replay.sh <playback_speed> <start_time>
```

## More Information

The Programming Manual explains the following topics in detail:

- WSL2, Ubuntu, and Docker setup
- Submitting programs to the Web simulator and checking results
- Detailed instructions for using the local simulator
- Sample program structure and editing procedures
- ROS APIs, movement, camera, and checkpoint processing
- Log design, error handling, and AR marker recognition

For details, see [docs/ProgrammingManual.md](docs/ProgrammingManual.md).

