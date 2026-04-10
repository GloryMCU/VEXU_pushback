# Basic 项目当前代码结构

本文档只描述仓库当前已经存在的目录和代码职责。

## 根目录

```text
basic/
├── CODE_STYLE.md
├── makefile
├── include/
├── src/
├── build/
└── vex/
```

- `makefile`：VEX V5 工程构建入口。
- `include/`：头文件。
- `src/`：实现文件。
- `build/`：构建产物目录。
- `vex/`：VEX 工具链相关 make 规则。

## 当前头文件结构

```text
include/
├── vex.h
├── control/
│   └── chassis.h
├── hardware/
│   ├── robot_config.h
│   └── sensors.h
├── input/
│   └── controller.h
├── executer/
└── output/
```

- `include/vex.h`
  - 引入 VEX SDK 头文件。
  - 目前还定义了 `COMPETITION` 宏。

- `include/control/chassis.h`
  - 声明 `basic::control::Chassis`。
  - 提供底盘控制、刹车、线程入口和机构控制相关接口声明。

- `include/hardware/robot_config.h`
  - 声明全局硬件对象和硬件常量。
  - 包括电机、控制器、惯导和刷新周期等配置。

- `include/hardware/sensors.h`
  - 声明传感器线程、IMU 相关函数、颜色识别状态和显示函数。

- `include/input/controller.h`
  - 声明 `ControllerState`。
  - 提供输入线程入口和输入快照读取接口。

- `include/executer/`
  - 当前为空目录。

- `include/output/`
  - 当前为空目录。

## 当前源文件结构

```text
src/
├── control/
│   ├── chassis.cpp
│   └── mechanisms.cpp
├── executer/
│   └── main.cpp
├── hardware/
│   ├── robot_config.cpp
│   └── sensors.cpp
├── input/
│   └── controller.cpp
└── output/
```

- `src/control/chassis.cpp`
  - 实现底盘功率整形、驾驶控制计算、刹车模式和底盘线程。

- `src/control/mechanisms.cpp`
  - 实现手动机构控制循环。
  - 负责上层、 middle、under 三个 overhang 电机，以及 intake/throw 机构模式切换。

- `src/executer/main.cpp`
  - 程序入口。
  - 负责惯导校准、后台线程启动和 `competition` 回调注册。

- `src/hardware/robot_config.cpp`
  - 定义 `robot_config.h` 里声明的硬件对象。
  - 包括底盘电机、机构电机、控制器、Brain 和 Inertial。

- `src/hardware/sensors.cpp`
  - 实现串口颜色传感器读取、加速标志更新、屏幕显示和 IMU 辅助函数。

- `src/input/controller.cpp`
  - 实现手柄输入采集线程。
  - 生成轴值、按钮状态和 `press_*` 边沿事件，并提供线程安全快照。

- `src/output/`
  - 当前为空目录。

## 当前命名空间

项目代码当前使用以下命名空间：

- `basic::control`
- `basic::hardware`
- `basic::input`

## 当前代码分层

- `hardware`
  - 放硬件对象定义和传感器相关逻辑。

- `input`
  - 放控制器输入采集和输入状态表达。

- `control`
  - 放底盘和机构的行为控制逻辑。

- `executer`
  - 放程序入口 `main.cpp`。

因此新代码目前应保持在这一层级深度内。
如果以后要继续增加更深目录，需要同步修改构建规则。

## 实际执行原则

创建新文件前先问自己：

“这是一个新的职责板块，还是已有职责的继续扩展？”

如果它不是新的职责板块，就优先放进当前已经存在、并且职责正确的模块里，而不是额外新建杂项文件。
