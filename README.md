# PID 云台实验室

面向嵌入式组的 C++ PID 入门实验。组员修改 **`src/tuning.cpp`**，在浏览器点击 **编译并运行 C++**，就能看到双轴机器人云台如何追踪目标、产生超调、抵抗外部负载。

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
3. 网页点击“编译并运行 C++”，观察 Yaw 超调和受负载后的误差。
4. 改回 `{0.8, 0.45, 0.22, 0.65, 0.025, true}`，再次编译，对比灰色的上次运行曲线。
5. 按 [中文实验手册](docs/实验手册.md) 完成 P → PI → PID → 积分饱和实验。

## 页面提供什么

- 双轴云台等轴测动画，以及 Yaw 俯视 / Pitch 侧视投影；虚线为目标。
- 实际角度、目标角度、上次运行角度曲线。
- P、I、D 分量和限幅后的电机力矩曲线。
- 暂停、回放、拖动时间轴、0.5× / 1× / 2× 播放、转轴切换。
- 初次阶跃超调、末秒平均绝对误差和 CSV 导出。
- 编译报错直接显示在页面，失败时保留上一次成功的结果。

网页输入框只改变实验目标与外力矩；PID 参数始终来自 C++ 文件。每次运行都重新编译并将仿真状态清零。

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
```

第一条检查控制器行为和**默认参数**的响应指标；自行调参后可能不满足默认性能断言，这是实验结果，不一定是程序错误。第二条独立编译固定课程参数，不改动学生的 `src/tuning.cpp`，验证实验现象并生成对照 CSV。

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
| 修改代码却没变化 | 保存 `src/tuning.cpp`，点击“编译并运行”，确认显示“编译成功”；检查当前观察的转轴 |
| 直接打开 index.html 无法运行 | 必须由 `python lab.py` 启动，页面通过本地服务调用 C++ |
| 编译失败 | 读页面错误；检查逗号、花括号、分号，布尔值写 `true` / `false` |
| 测试在改参数后失败 | 默认响应测试针对初始参数；先记录实验，再恢复初始参数复测 |

服务仅监听本机 `127.0.0.1`，每位组员在自己的电脑启动。这里是**离线数值仿真与回放**，并未连接真实硬件；模型限制和嵌入式迁移注意事项见实验手册。
