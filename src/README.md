# `basic/src` 框架说明

本文档面向第一次接手当前 `basic/src` 框架的同学，目标是帮助你快速理解：

- 这个框架现在包含什么
- 代码从哪里进入、状态怎么流动
- 新需求应该改哪一层
- 它和 `basic/old/VEXU_pushback` 的主要区别与优点

## 1. 框架整体定位

当前 `src/` 不是“把所有逻辑写进 `main.cpp`”的单体写法，而是一个按职责拆层的机器人框架。

它的核心思路是：

- 用统一的 `Robot` 接口描述“一台机器人”
- 用 `robot_selector` 选择当前编译要运行的机器人
- 每台机器人内部再拆成 `hardware`、`state`、`control`、`autonomous`
- 把通用输入处理和电机控制封装成公共模块，避免重复写底层细节

目前框架里已经有两套机器人实现：

- `basic_robot`
- `second_robot`

它们共用一套入口和公共基础件，但各自保留独立的硬件定义、状态结构、手动控制和自动程序。

## 2. 目录结构与职责

`src/` 当前主要分为下面几层：

```text
src/
├─ executer/                    程序入口
├─ hardware/                    机器人硬件定义、状态定义、机器人装配
│  ├─ basic_robot/
│  ├─ second_robot/
│  └─ shared/                   跨机器人共享状态类型
├─ control/                     控制逻辑
│  ├─ basic_robot/              basic_robot 专属控制
│  ├─ second_robot/             second_robot 专属控制
│  ├─ adrc/                     通用 ADRC 控制器基础件
│  ├─ kalman/                   通用 Kalman 滤波基础件
│  └─ motor_control.*           电机控制通用封装
└─ input/                       手柄输入采集与按键边沿处理
```

同时，`src/` 还依赖 `include/` 里的两个公共头文件：

- `include/app/robot.h`：统一机器人接口
- `include/hardware/robot_selector.h`：当前机器人选择器

## 3. 程序入口与运行流程

主入口在：

- `src/executer/main.cpp`

运行流程如下：

```text
main()
  -> get_current_robot()
  -> robot.initialize()
  -> robot.bind_background_tasks()
  -> robot.bind_competition(...)
```

也就是说，`main.cpp` 本身几乎不关心“这是哪台车、底盘怎么跑、机构怎么动”，它只负责：

- 选择当前机器人
- 调用统一生命周期接口
- 在比赛模式下绑定自动和手动回调

这也是当前框架最重要的设计点之一：入口稳定，机器人实现可替换。

## 4. 机器人选择机制

当前机器人选择在：

- `include/hardware/robot_selector.h`

关键点：

- `RobotIdentity` 枚举定义了可选机器人
- `kSelectedRobot` 决定当前编译使用哪台机器人
- `src/hardware/robot_selector.cpp` 负责把枚举映射到具体实例

如果你要切换当前运行的机器人，最直接的入口就是修改：

```cpp
inline constexpr RobotIdentity kSelectedRobot = RobotIdentity::kBasicRobot;
```

这比旧框架“直接换一套源码”更清晰，也更适合长期维护多台车。

## 5. 每台机器人的内部结构

以 `basic_robot` 为例，它的装配入口在：

- `src/hardware/basic_robot/basic_robot.cpp`

`second_robot` 对应入口在：

- `src/hardware/second_robot/second_robot.cpp`

每台机器人都实现同一个接口：

- `initialize()`：初始化硬件，当前主要是 IMU 标定
- `bind_background_tasks()`：启动后台线程
- `bind_competition()`：绑定自动程序和手动程序入口

这两个文件本质上是“机器人总装层”，负责把以下模块串起来：

- 硬件定义 `robot_hardware.h`
- 运行时状态 `robot_state.h`
- 传感器刷新 `sensors.cpp`
- 手柄输入 `input/controller.cpp`
- 手动控制 `control/.../chassis.cpp` 与 `mechanisms.cpp`
- 自动控制 `control/.../autonomous/routine.cpp`

建议把这两个 `*_robot.cpp` 理解为“总调度器”，而不是“业务实现区”。

## 6. 硬件层：端口定义集中管理

每台车都有自己的硬件定义文件：

- `src/hardware/basic_robot/robot_hardware.h`
- `src/hardware/second_robot/robot_hardware.h`

这里集中定义了：

- 电机端口
- 传感器端口
- 控制器对象
- 气动输出
- 刷新周期、死区等常量
- 硬件初始化辅助函数，比如 IMU 标定

新人接手时，如果要改端口、加设备、换电机方向，优先改这里，不要把端口号散落到控制逻辑里。

这也是相对旧框架的重要改进之一：硬件定义被收口到结构体中，而不是依赖全局设备变量四处访问。

## 7. 状态层：把“输入”和“控制目标”显式化

状态定义在：

- `src/hardware/shared/state_types.h`
- `src/hardware/basic_robot/robot_state.h`
- `src/hardware/second_robot/robot_state.h`

当前状态层主要包含：

- `ControllerInputState`：手柄轴值、按键状态、上一拍状态、按下沿事件
- `SensorState`：传感器相关状态
- `AutonomousState`：自动阶段的目标航向和位姿估计
- 各机器人自己的 `ChassisState`、`MechanismState`、`OverhangState`

这层的意义很大：

- 输入采样和业务控制解耦
- 控制逻辑不必直接操作控制器对象
- 自动和手动都可以共享状态
- 便于后续做日志、调试、回放和算法替换

和旧框架相比，当前代码不再依赖大量全局变量，例如 `A1/A2/A3/A4`、`L1/L2/R1/R2` 这种裸变量，而是统一放进状态结构里管理。

## 8. 输入层：统一处理手柄输入

输入更新模块在：

- `src/input/controller.h`
- `src/input/controller.cpp`

它负责：

- 读取摇杆轴值
- 读取按键状态
- 保存上一拍输入
- 生成 `press_x`、`press_l1` 这类按下沿事件
- 计算简单的输入变化率 `rating`

这样做的直接收益是：

- 底盘和机构控制都读同一份输入状态
- “按下触发一次”和“按住持续触发”被清晰区分
- 新人不用在每个控制函数里重复写按钮边沿判断

## 9. 控制层：按机器人拆分，按能力复用

### 9.1 通用控制模块

公共控制模块主要有：

- `src/control/motor_control.h`
- `src/control/motor_control.cpp`

它封装了：

- 速度控制
- 近似扭矩控制
- 停止模式控制
- 电机状态读取

好处是业务层只表达“我要多快、如何停”，不需要反复写 `motor.spin(...)` 和单位转换细节。

### 9.2 机器人专属控制模块

每台车的控制逻辑分开放在：

- `src/control/basic_robot/`
- `src/control/second_robot/`

典型拆分方式是：

- `chassis.*`：底盘手动控制
- `mechanisms.*`：机构手动控制
- `autonomous/routine.*`：自动程序

这种拆法的优点是：

- 公共层只保留共性
- 车型差异不会污染另一台车
- 新人能快速定位“我要改的是哪台车的哪块功能”

## 10. 自动程序现状

当前两台机器人都已经有独立自动程序：

- `src/control/basic_robot/autonomous/routine.cpp`
- `src/control/second_robot/autonomous/routine.cpp`

其中：

- `basic_robot` 的自动逻辑更复杂，已经包含基于 IMU 和轮子里程的简化位姿估计、`go_to_pose(...)`、激光测距停车等能力
- `second_robot` 的自动逻辑更接近旧框架思路，但也已经迁移到新的分层结构里

自动程序被独立放在 `autonomous/` 目录下，意味着：

- 自动代码不会再混在 `main.cpp`
- 机器人装配层只负责绑定，不负责实现动作细节
- 后续如果要扩展动作指令、做更完整的路径控制，落点很明确

`src/control/autonomous/README.md` 目前还保留了一份针对 `basic_robot` 自动逻辑的专项说明，可以和本文配合阅读。

## 11. 算法基础件现状

当前 `src/control/` 下还有两类通用算法模块：

- `src/control/adrc/`
- `src/control/kalman/`

需要明确一点：

- 这两个目录里的代码已经实现了通用控制器/滤波器基础件
- 但从当前主流程引用关系看，它们还没有正式接入 `basic_robot` 或 `second_robot` 的主控制回路

所以新人接手时应把它们理解为：

- 已存在、可复用的算法基础库
- 不是当前手动/自动流程的默认核心控制器

这类“先沉淀基础件，再逐步接主链路”的方式，也比旧框架更适合后续演进。

## 12. 新人对接时最常改的地方

如果你只是要接手维护，通常从下面几个位置开始就够了：

### 12.1 切换当前机器人

修改：

- `include/hardware/robot_selector.h`

### 12.2 改端口、方向、硬件配置

修改：

- `src/hardware/basic_robot/robot_hardware.h`
- `src/hardware/second_robot/robot_hardware.h`

### 12.3 改手动底盘逻辑

修改：

- `src/control/basic_robot/chassis.cpp`
- `src/control/second_robot/chassis.cpp`

### 12.4 改机构控制映射

修改：

- `src/control/basic_robot/mechanisms.cpp`
- `src/control/second_robot/mechanisms.cpp`

### 12.5 改自动程序

修改：

- `src/control/basic_robot/autonomous/routine.cpp`
- `src/control/second_robot/autonomous/routine.cpp`

### 12.6 加新的传感器状态或后台任务

优先看：

- `src/hardware/*/sensors.*`
- `src/hardware/*/*_robot.cpp`
- `src/hardware/shared/state_types.h`

## 13. 当前框架相对 `basic/old/VEXU_pushback` 的主要区别

旧框架的典型特点是：

- 入口、手动控制、自动控制耦合比较紧
- 大量依赖全局设备对象和全局输入变量
- 控制函数以“过程式函数集合”为主
- 默认只服务单一机器人
- 目录层级较浅，模块边界不够清晰

当前 `src/` 框架的主要区别如下。

### 13.1 从“单车单套代码”变成“统一入口 + 多机器人实现”

旧框架本质上只面向一台具体机器人。

新框架通过：

- `Robot` 抽象接口
- `RobotIdentity`
- `robot_selector`

把“运行哪台机器人”变成显式配置，而不是靠复制代码或手动删改。

### 13.2 从全局变量驱动改成状态对象驱动

旧框架中常见写法是：

- 设备对象全局暴露
- 摇杆量和按钮量全局暴露
- 控制函数直接读写这些全局量

新框架改成：

- 硬件放进 `RobotHardware`
- 运行时数据放进 `RobotState`
- 输入统一写入 `ControllerInputState`

这样依赖关系更清楚，也更容易调试和扩展。

### 13.3 从“功能堆在一起”改成按层拆分

旧框架里 `main.cpp`、`controller.cpp`、`differential-base.cpp`、`robot-config.cpp` 彼此联系紧密，新人需要先记住很多隐式约定。

新框架把职责拆开为：

- 入口层
- 机器人装配层
- 硬件层
- 状态层
- 输入层
- 控制层
- 自动层

拆层后，代码的阅读路径和修改路径都更稳定。

### 13.4 从“直接操作底层对象”改成“公共能力封装”

旧框架里很多控制行为直接调用底层设备接口。

新框架已经抽出：

- `motor_control`
- `controller_update`
- 共用状态类型

这样可以减少重复代码，也便于后续统一更改控制策略。

### 13.5 自动程序的组织方式更清晰

旧框架的自动函数直接写在主工程流程附近。

新框架中：

- 自动程序按机器人分文件
- 绑定和实现分离
- 自动状态被纳入 `RobotState`

对后续做更长、更复杂的自动流程更友好。

## 14. 当前框架的实际优点

相对 `basic/old/VEXU_pushback`，当前框架的优点可以概括为下面几条。

### 14.1 更适合多人协作

因为目录和职责更清晰，不同人可以相对独立地修改：

- 一人改硬件配置
- 一人改手动控制
- 一人改自动程序

发生冲突的概率会比旧框架低很多。

### 14.2 更适合维护多台机器人

现在同一个工程下已经能同时容纳两台机器人实现，并复用公共入口和基础模块。以后如果再加第三台车，也有明确落点。

### 14.3 更适合持续迭代控制算法

因为硬件、状态和控制层已经拆开，后续要把：

- PID
- ADRC
- Kalman
- 更复杂的定位与路径控制

接进来时，不需要把整套工程重写一遍。

### 14.4 更容易定位问题

出现问题时，可以先判断问题属于哪一层：

- 端口或方向错误：看 `robot_hardware.h`
- 手柄映射错误：看 `input/controller.cpp` 或 `mechanisms.cpp`
- 底盘控制异常：看 `chassis.cpp`
- 自动动作异常：看 `autonomous/routine.cpp`

这比在旧框架里全局搜函数、追全局变量要高效得多。

### 14.5 更容易做规范化开发

当前代码已经有明显的模块边界和命名空间边界，这让下面这些工作变得更现实：

- 单独评审某一层代码
- 给某一层补测试或日志
- 给不同机器人建立统一接口规范
- 逐步替换旧实现而不破坏整个工程

## 15. 当前框架也还存在的现实情况

为了避免误解，新人还需要知道当前框架并不是“全部都已经完全成熟”。

目前比较明显的现状有：

- `basic_robot` 的 `sensors.cpp` 还比较轻，很多传感器逻辑仍有继续补全空间
- `second_robot` 的 `sensors.cpp` 目前基本是空实现
- `adrc/` 和 `kalman/` 已存在，但还未正式接入主控制回路
- 两台机器人虽然共享总框架，但控制策略仍然各自演化，公共抽象还可以继续收敛

这并不是问题，反而说明当前框架已经从“能跑”迈到了“能继续扩展”的阶段。

## 16. 推荐的阅读顺序

第一次接手时，建议按这个顺序读代码：

1. `src/executer/main.cpp`
2. `include/hardware/robot_selector.h`
3. `src/hardware/robot_selector.cpp`
4. 你当前要维护的那台车的 `src/hardware/*_robot/*.cpp`
5. 对应机器人的 `robot_hardware.h` 和 `robot_state.h`
6. 对应机器人的 `chassis.cpp`、`mechanisms.cpp`
7. 对应机器人的 `autonomous/routine.cpp`
8. 最后再看 `motor_control`、`adrc`、`kalman` 这些通用基础件

这样最容易先建立“主流程视角”，再深入局部实现。

## 17. 一句话总结

`basic/src` 当前已经不是旧版那种围绕单台机器人、全局变量和过程函数堆起来的工程，而是一套面向多机器人、分层组织、便于扩展和维护的框架。

对于新人来说，最重要的不是一次性记住所有文件，而是先理解这套分层：

- 入口负责启动
- 装配层负责串模块
- 硬件层负责设备定义
- 状态层负责数据承载
- 控制层负责行为实现
- 自动层负责动作流程

理解了这条主线，后续无论是改端口、改手动、改自动，还是接新算法，都会顺很多。
