# README Update Log

本文件用于记录每次代码修改的更新日志。

后续我在这个仓库里改代码时，会同步补充以下内容：
- 修改了哪些文件
- 为什么改
- 当前还存在什么问题或限制
- 后续如果继续改，应该从哪里下手

---

## 2026-04-06 APP 层拆分

### 1. 本次修改目标
- 新增 `User_File/3_APP` 应用层目录。
- 将云台和底盘的业务逻辑从 `Task` 层拆出。
- 让 `Task` 层只保留硬件回调、初始化调度、RTOS 任务转发。

### 2. 修改文件

#### 新增文件
- `User_File/3_APP/Gimbal/app_gimbal.h`
- `User_File/3_APP/Gimbal/app_gimbal.cpp`
- `User_File/3_APP/Chassis/app_chassis.h`
- `User_File/3_APP/Chassis/app_chassis.cpp`

#### 修改文件
- `User_File/4_Task/tsk_config_and_callback.cpp`

### 3. 具体改了什么

#### 3.1 云台应用层
位置：
- `User_File/3_APP/Gimbal/app_gimbal.h`
- `User_File/3_APP/Gimbal/app_gimbal.cpp`

修改内容：
- 新建云台应用层接口。
- 将原先放在 `tsk_config_and_callback.cpp` 里的 GM6020 初始化迁移到 `App_Gimbal_Init()`。
- 将原先放在 1ms 回调里的云台电机控制和 Kalman 预测/更新迁移到 `App_Gimbal_Task_1ms_Callback()`。
- 将 VOFA 下发的 `Q/R` 调参逻辑迁移到 `App_Gimbal_Vofa_Set_Debug_Variable()`。
- 将 CAN ID `0x206` 的接收回调转发到 `App_Gimbal_CAN_RxCpltCallback()`。

#### 3.2 底盘应用层
位置：
- `User_File/3_APP/Chassis/app_chassis.h`
- `User_File/3_APP/Chassis/app_chassis.cpp`

修改内容：
- 新建底盘应用层接口。
- 将 4 个 STW 电机初始化迁移到 `App_Chassis_Init()`。
- 将原来 `RTOS_Ctrl_Task_Loop()` 里的遥控差速混控迁移到 `App_Chassis_Ctrl_Task_Loop()`。
- 将原来 VOFA 四轮监控打包逻辑迁移到 `App_Chassis_Monitor_Task_Loop()`。
- 将 STW 电机 CAN 回调转发统一到 `App_Chassis_CAN_RxCpltCallback()`。

#### 3.3 Task 层收口
位置：
- `User_File/4_Task/tsk_config_and_callback.cpp`

修改内容：
- 删掉原来直接持有的云台电机、底盘电机、Kalman 矩阵对象。
- `Task_Init()` 里改为调用 `App_Gimbal_Init()` 和 `App_Chassis_Init()`。
- `Task1ms_Callback()` 里改为调用 `App_Gimbal_Task_1ms_Callback()`。
- `CAN1_Callback()` 里改为按 ID 分发到应用层。
- `RTOS_Ctrl_Task_Loop()` 改为直接调用 `App_Chassis_Ctrl_Task_Loop()`。
- `RTOS_Monitor_Task_Loop()` 改为直接调用 `App_Chassis_Monitor_Task_Loop()`。

### 4. 这次修改主要解决的问题

#### 问题 1：`tsk_config_and_callback.cpp` 职责过重
现象：
- 同一个文件里同时放了硬件回调、任务调度、云台控制、底盘控制、VOFA 调参、VOFA 监控。

影响：
- 后续继续加射击、拨杆模式、云台闭环、底盘模式时，这个文件会继续膨胀。
- 修改一个功能时容易误伤别的逻辑。

本次处理：
- 把“功能逻辑”迁到了 `3_APP`。
- 把 `4_Task` 收敛成“调度层”。

#### 问题 2：应用逻辑没有独立层
现象：
- 当前工程有 `Middleware / Device / Task`，但没有直接表达机器人业务功能的层。

影响：
- 云台、底盘、射击等机器人功能无法按模块组织。
- 代码结构不利于多人协作。

本次处理：
- 新增 `User_File/3_APP`，作为应用层。

### 5. 当前已知问题 / 限制

#### 问题 1：云台应用目前还是“单电机测试态”
位置：
- `User_File/3_APP/Gimbal/app_gimbal.cpp`

现状：
- `App_Gimbal_Task_1ms_Callback()` 里仍然是固定目标角度 `PI` 的测试逻辑。

影响：
- 现在只是完成了层级迁移，没有形成真正的云台应用控制链。

后续怎么改：
- 在 `app_gimbal.cpp` 里增加模式状态、目标角来源、遥控输入映射。
- 把固定角度改成由遥控器、上位机或视觉输入驱动。

#### 问题 2：底盘应用还没有模式机
位置：
- `User_File/3_APP/Chassis/app_chassis.cpp`

现状：
- 当前只有基础差速控制。
- 还没有“停机 / 遥控 / 自瞄协同 / 急停 / 调试”等模式切换。

影响：
- 后续功能扩展时，控制逻辑会继续堆在 `App_Chassis_Ctrl_Task_Loop()`。

后续怎么改：
- 在底盘 APP 层增加枚举状态或 FSM。
- 将遥控解析、目标解算、输出控制拆成多个内部函数。

#### 问题 3：目前没有完成编译验证
现状：
- 本地运行 `cmake -S . -B build` 失败。
- 原因是当前环境缺少 `nmake` 和对应 C/C++ 编译器配置。

影响：
- 这次变更完成了结构整理，但还没在本机完成最终编译确认。

后续怎么改：
- 在已配置 STM32 工具链的环境中重新 `Configure + Build`。
- 优先检查新增 `3_APP` 文件是否都被 CMake 收集并参与编译。

### 6. 后续继续修改时建议入口

#### 如果要继续改云台
优先修改：
- `User_File/3_APP/Gimbal/app_gimbal.h`
- `User_File/3_APP/Gimbal/app_gimbal.cpp`

建议方向：
- 增加 `Set_Target`、模式切换、遥控输入、IMU 解算耦合。

#### 如果要继续改底盘
优先修改：
- `User_File/3_APP/Chassis/app_chassis.h`
- `User_File/3_APP/Chassis/app_chassis.cpp`

建议方向：
- 增加底盘模式机。
- 把遥控输入处理和电机输出进一步解耦。

#### 如果要继续改任务调度
优先修改：
- `User_File/4_Task/tsk_config_and_callback.cpp`

建议方向：
- 保持这里只做“调度”和“回调分发”。
- 不再把业务算法直接写回 `Task` 层。

### 7. 后续日志记录格式

以后每次我改代码，会按下面格式继续追加：

```md
## 日期 + 主题

### 1. 本次修改目标

### 2. 修改文件

### 3. 具体改了什么

### 4. 解决了什么问题

### 5. 当前遗留问题

### 6. 后续怎么改
```

---

## 2026-04-06 application 外置重构

### 1. 本次修改目标
- 删除 `User_File/3_APP` 这一层命名。
- 在工程最外层新增统一的 `application/` 业务应用层。
- 按你指定的目录结构预留业务模块入口。
- 让 CMake 能正确编译根目录下的 `application/` 源码。

### 2. 修改文件

#### 新增文件
- `application/chassis/app_chassis.h`
- `application/chassis/app_chassis.cpp`
- `application/gimbal/app_gimbal.h`
- `application/gimbal/app_gimbal.cpp`
- `application/assist/.gitkeep`
- `application/calibrate/.gitkeep`
- `application/communication/.gitkeep`
- `application/custom_controller/.gitkeep`
- `application/IMU/.gitkeep`
- `application/mechanical_arm/.gitkeep`
- `application/music/.gitkeep`
- `application/other/.gitkeep`
- `application/referee/.gitkeep`
- `application/remote_control/.gitkeep`
- `application/robot_cmd/.gitkeep`
- `application/shoot/.gitkeep`
- `application/typedef/.gitkeep`

#### 修改文件
- `CMakeLists.txt`
- `README_UPDATE_LOG.md`

#### 删除内容
- `User_File/3_APP` 下原有应用层文件将迁移后删除。

### 3. 具体改了什么

#### 3.1 application 目录外置
位置：
- 工程根目录 `application/`

修改内容：
- 将原先位于 `User_File/3_APP` 的应用层移动到根目录。
- 底盘模块放到 `application/chassis/`。
- 云台模块放到 `application/gimbal/`。

#### 3.2 预留完整业务目录
位置：
- `application/assist/`
- `application/calibrate/`
- `application/communication/`
- `application/custom_controller/`
- `application/IMU/`
- `application/mechanical_arm/`
- `application/music/`
- `application/other/`
- `application/referee/`
- `application/remote_control/`
- `application/robot_cmd/`
- `application/shoot/`
- `application/typedef/`

修改内容：
- 先创建目录占位，避免后续扩展时再动整体结构。

#### 3.3 构建系统适配
位置：
- `CMakeLists.txt`

修改内容：
- 在源码递归收集里加入 `application/**/*.c` 和 `application/**/*.cpp`。
- 在头文件 include 目录递归收集里加入 `application/**/*.h`。

### 4. 这次修改解决的问题

#### 问题 1：应用层放在 `User_File/3_APP` 不符合你要的最终工程结构
本次处理：
- 改成根目录统一 `application/`。

#### 问题 2：根目录新增源码默认不会被当前 CMake 编译
本次处理：
- 已同步修改 `CMakeLists.txt`，避免新目录创建后代码不参与构建。

### 5. 当前遗留问题

#### 问题 1：旧目录删除后，IDE 可能需要刷新
现象：
- 一些编辑器会暂时保留旧目录缓存。

后续怎么改：
- 刷新工程树。
- 如使用 CMake Tools，重新执行一次 `Configure`。

#### 问题 2：目录现在只是结构预留
现状：
- 目前只有 `chassis` 和 `gimbal` 有实际代码。
- 其他目录只是预留空位。

后续怎么改：
- 后续新增模块时，直接放到对应子目录。
- 保持 `Task` 层只做调度，不把业务逻辑再写回 `User_File/4_Task`。

### 6. 后续建议
- 遥控相关逻辑后续可以从底盘模块继续拆到 `application/remote_control/`。
- 电机目标下发和执行器命令可以再从 `chassis/gimbal` 抽到 `application/robot_cmd/`。
- IMU 融合、姿态解算后续可以从当前 BSP/Task 过渡到 `application/IMU/`。
