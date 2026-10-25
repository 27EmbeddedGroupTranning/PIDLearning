# PID 云台实验室

面向嵌入式组的 C++ PID 入门实验。组员修改 **`src/tuning.cpp`**，在浏览器点击 **重新编译 C++**，随后拖动 Yaw / Pitch 滑块，实时观察双轴机器人云台追踪目标、产生超调、抵抗外部负载。

控制器和物理仿真都在 C++ 中执行，浏览器负责显示结果。无需开发板、外部 CDN、npm 或 Python 第三方库。

## 快速开始

准备 **Python 3.9+**、支持 **C++11 的 g++ 或 clang++** 和现代浏览器。使用 Git 克隆时还需要 Git。Windows 可以使用已有的 MinGW-w64 工具链；Linux 使用发行版的 g++；macOS 使用 Xcode Command Line Tools 中的 clang++。编译器必须能从当前终端访问。

组长发布后，将下面的 `<仓库URL>` 替换为实际地址；当前文件夹已经包含完整项目，也可直接从第 2 步开始。

```bash
# 1. 获取项目
git clone <仓库URL> pid-gimbal-lab
cd pid-gimbal-lab

# 2. 检查环境（macOS / Linux 中 python 可能需要写成 python3）
python --version
g++ --version

# 3. 启动，自动打开浏览器
python lab.py
```

只有 clang++ 时，用 `clang++ --version` 检查；启动器会自动寻找 g++ 或 clang++。

浏览器访问 **http://127.0.0.1:8000**。终端保持打开，按 `Ctrl+C` 退出。

1. 打开 `src/tuning.cpp`。
2. 将 `yaw_gains()` 中的参数改为 `{0.8, 0, 0, 0.65, 0.025, true}`，保存。
3. 网页点击“重新编译 C++”，拖动 Yaw 滑块到 45°，把外力矩设为 0.08，观察超调和负载误差。
4. 改回 `{0.8, 0.45, 0.22, 0.65, 0.025, true}`，再次编译，以相同目标和负载比较响应。每次实验可导出 CSV 留存。
5. 按 [中文实验手册](docs/实验手册.md) 完成 P → PI → PID → 积分饱和实验。

## 页面提供什么

- Yaw（±150°）、Pitch（±60°）目标滑块，拖动即发送指令；目标归零按钮。
- 持续运行的双轴云台动画、实际与目标角度、P/I/D 和限幅后的力矩。
- 最近 15 秒滚动曲线、最近 1 秒平均绝对误差、CSV 导出。
- 暂停 / 继续仿真、重置角度与积分状态、切换观察转轴。
- 修改 C++ 后重新编译；编译失败时原控制器继续运行并显示错误。

网页角度和外力矩指令通过 Python 本地服务送入持久运行的 C++ 进程。C++ 保存两轴的角度、速度和积分，以 2 ms 固定步长计算，每 20 ms 接收最新指令并推进 10 步；浏览器约每 50 ms 获取新增数据。拖动滑块不会重置控制器，也不需要重新编译。

这是桌面环境的实时交互仿真，不保证硬实时调度：系统繁忙时仿真时间可能落后于墙钟时间。暂停会冻结物理与控制状态；重置和成功重新编译会清空状态与曲线、保留当前目标、负载及暂停状态。页面刷新连接现有进程，不重新启动仿真。同一服务的多个标签页共享同一云台。

## 文件导航

```text
src/tuning.cpp          ← 学生主要修改这里：两轴 PID 参数
include/pid.hpp         ← PID 算法实现：进阶实验可修改
src/main.cpp            ← 500 Hz 控制循环、双轴物理模型、CSV 输出
lab.py                  ← 编译、运行和本地 HTTP 服务
web/                    ← 云台动画、曲线和页面
tests/                  ← 控制器测试、课程参数对照验证
docs/实验手册.md         ← 原理、操作步骤、思考题
docs/实验记录模板.md     ← 学生填写、提交
docs/组长教学与发布.md   ← 授课安排和 Git 发布步骤
CMakeLists.txt          ← 可选的纯 C++ 构建入口
build/                  ← 自动生成的程序和 CSV，不纳入版本控制
```

## 验证与纯命令行运行

```bash
python lab.py --test
python tests/verify_lessons.py
python tests/test_live.py
```

第一条检查控制器行为和**默认参数**的响应指标；自行调参后可能不满足默认性能断言，这是实验结果，不一定是程序错误。第二条独立编译固定课程参数，不改动学生的 `src/tuning.cpp`，验证离线实验现象并生成对照 CSV。第三条验证实时指令、状态连续性、暂停、重置、编译失败恢复与 HTTP 接口。

可选 CMake 构建不依赖 Python：

```bash
cmake -S . -B build/cmake
cmake --build build/cmake --config Release
ctest --test-dir build/cmake -C Release --output-on-failure
```

在 Windows MinGW 环境，如默认生成器要求 Visual Studio，可改用：

```bash
cmake -S . -B build/mingw -G "MinGW Makefiles"
cmake --build build/mingw
ctest --test-dir build/mingw --output-on-failure
```

构建出的 `gimbal_sim` 接受 `Yaw角度 Pitch角度 外力矩 是否反向` 四个位置参数，例如 `gimbal_sim 45 20 0.08 0`；结果写到标准输出。Windows 程序为 `.exe`，多配置生成器通常位于 `Release` 目录。浏览器启动器使用自己的 `build/gimbal_sim`，不依赖 CMake 的输出。

## 常见问题

| 现象 | 处理方法 |
| --- | --- |
| 找不到 python | Windows 可试 `py -3 lab.py`；macOS / Linux 可试 `python3 lab.py` |
| 找不到编译器 | 确认 g++ 或 clang++ 在 PATH 中，重新打开终端；也可设置 `CXX` 为编译器的完整路径（不要附加编译选项） |
| 端口被占用 | `python lab.py --port 8001` |
| 自动打开浏览器失败 | 手动访问终端显示的地址，或使用 `--no-browser` |
| Windows 提示拒绝访问 / 程序被隔离 | 检查安全软件的隔离记录，确认是本仓库生成的程序后依照所在组织规则处理；无需关闭防护 |
| 修改代码却没变化 | 保存 `src/tuning.cpp`，点击“重新编译 C++”，确认没有错误；检查当前观察的转轴 |
| 直接打开 index.html 无法运行 | 必须由 `python lab.py` 启动，页面通过本地服务调用 C++ |
| 编译失败 | 读页面错误；检查逗号、花括号、分号，布尔值写 `true` / `false` |
| 测试在改参数后失败 | 默认响应测试针对初始参数；先记录实验，再恢复初始参数复测 |

服务仅监听本机 `127.0.0.1`，每位组员在自己的电脑启动。这里是**由 C++ 持续计算的交互式数值仿真**，并未连接真实硬件；模型限制和嵌入式迁移注意事项见实验手册。
