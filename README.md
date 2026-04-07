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

---

## 2026-04-06 编译修复: struct_typedef 类型冲突 + 链接错误 + inline 警告

### 1. 本次修改目标

- 修复 `struct_typedef.h` 手动重定义 `int32_t`/`uint32_t` 与 ARM GCC `<stdint.h>` 冲突导致的编译错误
- 修复 `remote_control.h` 缺少 `extern "C"` 导致 C++ 翻译单元链接时找不到 C 符号
- 消除 `chassis.h` / `gimbal.h` 中 `inline` 函数声明但无定义产生的所有警告

### 2. 修改文件

| 文件                                   | 修改说明                                                                                           |
| -------------------------------------- | -------------------------------------------------------------------------------------------------- |
| `application/typedef/struct_typedef.h` | 移除手动 typedef 的 `int8_t`~`uint64_t`，改为 `#include <stdint.h>`，只保留 `bool_t`/`fp32`/`fp64` |
| `application/other/remote_control.h`   | 添加 `extern "C" { }` 包裹，使 C 函数在 C++ 编译单元正确链接                                       |
| `application/chassis/chassis.h`        | `inline` 声明改为普通函数声明（无定义体时 `inline` 在 C 中触发 "declared but never defined" 警告） |
| `application/gimbal/gimbal.h`          | 同上；同时将声明移入 `#if GIMBAL_TYPE != GIMBAL_NONE` 守卫内                                       |

### 3. 具体 bug 分析

#### Bug 1: `struct_typedef.h` 类型冲突 (编译错误)

**现象**：

```
error: conflicting types for 'int32_t'; have '__int32_t' {aka 'long int'}
note: previous declaration of 'int32_t' with type 'int32_t' {aka 'int'}
```

**原因**：这是从 F4 工程复制来的头文件。在 STM32F4 的 arm-none-eabi-gcc 上 `int` 和 `long` 在 32 位 ARM 上恰好都是 4 字节且被视为兼容类型，所以不报错。但 STM32H7 使用的 GCC 13.3.1 严格区分 `signed int` 和 `long int`：

- `struct_typedef.h`: `typedef signed int int32_t;` → 底层类型是 `int`
- `<stdint.h>`: `typedef __int32_t int32_t;` → 底层类型是 `long int`

两者虽然大小相同，但 C/C++ 标准认为它们是不同类型。

**修复**：删除所有手动 typedef，直接 `#include <stdint.h>`。

#### Bug 2: `remote_control_init()` 链接错误 (undefined reference)

**现象**：

```
undefined reference to `remote_control_init()'
undefined reference to `get_remote_control_point()'
```

**原因**：`remote_control.c` 是 C 文件，函数以 C 链接（无 name mangling）编译。但 `remote_control.h` 没有 `extern "C"` 保护，被 C++ 的 `tsk_config_and_callback.cpp` include 时，编译器按 C++ mangled 名查找符号，链接器自然找不到。

**修复**：在 `remote_control.h` 添加 `#ifdef __cplusplus extern "C" { #endif` 包裹。

#### Bug 3: inline 函数警告

**现象**：

```
warning: inline function 'ChassisGetStatus' declared but never defined
warning: inline function 'GetGimbalStatus' declared but never defined
```

（共 12 个警告）

**原因**：`chassis.h` 和 `gimbal.h` 在 `#if xxx_TYPE != xxx_NONE` 守卫**内**声明了 `inline` 函数，但由于当前 `CHASSIS_TYPE == CHASSIS_NONE` / `GIMBAL_TYPE == GIMBAL_NONE`，.c 文件里的定义体也被 `#if` 跳过，导致只有声明没有定义。GCC 的 GNU inline 语义下这会触发警告。更深层的问题是 `gimbal.h` 的声明放在了 `#if` 守卫**之外**。

**修复**：

- 将 `inline` 改为普通函数声明（后续实现时再加 `inline` + 定义体）
- 将 `gimbal.h` 的声明移入 `#if GIMBAL_TYPE != GIMBAL_NONE` 守卫内

### 4. 编译结果

```
Build: 0 errors, 0 warnings
FLASH: 165852 B / 1 MB (15.82%)
RAM_D1: 317872 B / 320 KB (97.01%)
```

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

---

## 2026-04-06 bsp_rc 按 H723 + UART5 重构

### 1. 本次修改目标

- 将 `bsp_rc` 从旧的 `USART3` 假设改为当前工程实际使用的 `UART5`。
- 按 STM32H723 的 UART/DMA 方式重写遥控底层接收。
- 保留 DJI DBUS 的关键串口配置说明，避免因为校验位/字长配置错误导致解包异常。

### 2. 修改文件

- `User_File/2_Device/Remote/bsp_rc.h`
- `User_File/2_Device/Remote/bsp_rc.c`

### 3. 具体改了什么

#### 3.1 底层串口从 USART3 改为 UART5

位置：

- `User_File/2_Device/Remote/bsp_rc.c`

修改内容：

- 底层句柄从 `huart3` / `hdma_usart3_rx` 改为 `huart5` / `hdma_uart5_rx`。
- DMA 寄存器访问改为 `UART5` 当前工程对应的 RX DMA 通道。

#### 3.2 保留 H723 上 DBUS 的正确字长/校验配置

位置：

- `User_File/2_Device/Remote/bsp_rc.h`
- `User_File/2_Device/Remote/bsp_rc.c`

修改内容：

- 增加注释，明确 DJI DBUS 在 STM32H723 HAL 上必须配置成：
  - `100000 baud`
  - `UART_WORDLENGTH_9B`
  - `UART_PARITY_EVEN`
  - 默认 `UART_STOPBITS_1`
- 明确说明：如果错误配置成 `8B + Even`，有效数据会变成 7 位，DBUS 解包会错位。

#### 3.3 为后续兼容 SBUS/i6x 预留停止位宏

位置：

- `User_File/2_Device/Remote/bsp_rc.h`

修改内容：

- 新增 `BSP_RC_UART_STOPBITS` 宏，默认值是 `UART_STOPBITS_1`。
- 如果后续不是 DJI DBUS，而是按当前 `i6x` 这类 SBUS 链路复用 `UART5`，只需要改成 `UART_STOPBITS_2`。

### 4. 这次修改解决的问题

#### 问题 1：原 `bsp_rc` 假定工程走 USART3，但当前工程实际遥控链路在 UART5

本次处理：

- 已切换到底层 `UART5`。

#### 问题 2：DBUS 在 H723 上最容易出错的是字长和校验位

本次处理：

- 已把 `9B + Even` 的关键原因写进 BSP 注释和配置中，避免后面重复踩坑。

### 5. 当前遗留问题

#### 问题 1：当前工程里 `i6x` 用的是 SBUS，不是 DJI DBUS

现状：

- `i6x` 当前链路说明的是 `UART5 100k 9E2`，偏向 SBUS。
- 这次 `bsp_rc` 默认是按 DJI DBUS 的 `1 stop bit` 处理。

后续怎么改：

- 如果你最终接的是 SBUS 接收机，改 `BSP_RC_UART_STOPBITS` 为 `UART_STOPBITS_2`。
- 如果接的是 DJI DBUS，保持默认 `UART_STOPBITS_1`。

#### 问题 2：目前只重构了 `bsp_rc`，上层遥控解析文件还没有一起统一

后续怎么改：

- 后面如果要彻底收口，建议把 `bsp_rc` 和 `i6x` / `remote_control` 的接收入口统一成一套回调接口。

### 6. 追加修正：bsp_rc 不修改底层

- 根据后续要求，`bsp_rc` 已再次收口为“纯 BSP 适配层”。
- 现在它不再在内部修改 `UART5`、DMA、寄存器或 HAL 初始化参数。
- 底层串口配置必须继续由 CubeMX、`drv_uart` 或工程现有串口接收链路负责。
- `bsp_rc` 当前只提供：
  - 接收缓冲区登记
  - 帧回调注册
  - `RC_Process_Received_Data()` 数据转发入口
  - 使能/失能状态管理

涉及文件：

- `User_File/2_Device/Remote/bsp_rc.h`
- `User_File/2_Device/Remote/bsp_rc.c`

### 7. 追加修复：NULL 编译问题

- `bsp_rc.c` 中使用了 `NULL`，但之前没有包含 `<stddef.h>`。
- 这会导致部分编译环境下出现 `NULL undeclared` 或同类编译错误。
- 已在 `User_File/2_Device/Remote/bsp_rc.c` 中补充 `<stddef.h>`。
- 同时修正了 `RC_Init()` 参数非法时的软件状态，避免初始化失败后残留旧缓冲区和启用状态。

### 8. 追加整理：bsp_rc 按 i6x 风格重写

- 重新整理了 `User_File/2_Device/Remote/bsp_rc.h`
- 重新整理了 `User_File/2_Device/Remote/bsp_rc.c`

这次整理的目标：

- 让 `bsp_rc` 和 `i6x` 一样更容易读
- 注释更直白
- 接口职责更清楚
- 不再混入底层串口配置逻辑

具体调整：

- 头文件中补充了完整职责说明和逐个接口注释
- 实现文件改成“文件说明 + 少量静态状态 + 明确的函数边界”
- 新增 `RC_Buffer_Config_Is_Valid()`，把缓冲区合法性检查单独收口
- `RC_Process_Received_Data()` 现在会明确校验：
  - 是否已启用
  - 缓冲区配置是否有效
  - 输入指针和长度是否合法
  - 输入缓冲区是否属于登记过的双缓冲区

当前好处：

- 逻辑比之前更线性
- 维护时更容易判断每个函数在做什么
- 和 `i6x.c` 的“单职责 + 直接判断 + 早返回”风格更一致

---

## 2026-04-06 遥控器驱动全栈适配达妙MC02 (UART5 + drv_uart)

### 1. 本次修改目标

- 将遥控器驱动从 F4 的 USART3 直接 DMA 寄存器操作，**完整迁移**到达妙 MC02 (STM32H723) 的 UART5 + `drv_uart` 中间件架构。
- 彻底解决 DJI DT7 遥控器与 SBUS 类遥控器在达妙板上的**校验差异**（停止位 / RX 反相）。
- 适配 `application/other/remote_control.c/h`，移除所有 F4 特有依赖，使其在 H7 上可编译运行。
- 保持上层 API（`GetDt7RcCh()` / `GetRcOffline()` 等）不变，对应用层透明。

### 2. 修改文件

#### 重写文件

| 文件                                   | 说明                                                                                                                                          |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `User_File/2_Device/Remote/bsp_rc.h`   | BSP 层头文件：新增 `RC_RxCallback_t` 回调类型，移除旧 F4 DMA 缓冲区参数                                                                       |
| `User_File/2_Device/Remote/bsp_rc.cpp` | **C→C++ 重命名**，内部调用 `drv_uart` 的 `UART_Init()` / `UART_Reinit()`；按 `__RC_TYPE` 动态修正 UART5 停止位                                |
| `application/other/remote_control.c`   | 移除 `USART3_IRQHandler` / `detect_hook` / `bsp_usart.h` / `sbus_to_usart1` 等 F4 依赖；新增 `remote_control_rx_callback()` 作为 DMA 接收入口 |
| `application/other/remote_control.h`   | 移除 `sbus_to_usart1` 声明，新增 `remote_control_rx_callback` 声明                                                                            |

#### 修改文件

| 文件                                           | 说明                                                           |
| ---------------------------------------------- | -------------------------------------------------------------- |
| `User_File/4_Task/tsk_config_and_callback.cpp` | 删除旧 `UART5_SBUS_Callback`，改为调用 `remote_control_init()` |

#### 备份文件（不参与编译）

| 文件                                            | 说明                           |
| ----------------------------------------------- | ------------------------------ |
| `User_File/2_Device/Remote/bsp_rc_old_f4.c.bak` | 旧 F4 版本 bsp_rc 备份         |
| `application/other/remote_control_old_f4.c.bak` | 旧 F4 版本 remote_control 备份 |

### 3. 具体改了什么

#### 3.1 bsp_rc 完全重写 (C→C++)

位置：`User_File/2_Device/Remote/bsp_rc.cpp` + `bsp_rc.h`

核心变更：

- **从直接操作 DMA 寄存器 → 通过 `drv_uart` 中间件**
  - 旧代码手动操作 `DMA_SxCR_CT` / `M0AR` / `M1AR` / `NDTR` 等 F4 DMA 寄存器
  - 新代码调用 `UART_Init(&huart5, callback)` 和 `UART_Reinit(&huart5)`，由 `drv_uart` 统一管理 DMA 双缓冲
- **DT7 vs SBUS 自动适配**：
  ```
  ┌──────────┬───────┬──────┬──────────┬───────────┐
  │ 遥控器   │ 波特率│ 校验 │ 停止位   │ RX反相    │
  ├──────────┼───────┼──────┼──────────┼───────────┤
  │ DJI DT7  │100000 │ 8E   │ 1 stop   │ 否        │
  │ SBUS类   │100000 │ 8E   │ 2 stop   │ 板载硬件  │
  └──────────┴───────┴──────┴──────────┴───────────┘
  ```
  CubeMX 默认配置为 SBUS (2 stop)，若 `__RC_TYPE == RC_DT7`，`RC_Init()` 会在运行时修正为 1 stop + 关闭 RX 反相。
- **extern "C" 导出**：C++ 实现但通过头文件的 `extern "C"` 导出 C 链接符号，remote_control.c (C) 可直接调用。

#### 3.2 remote_control.c 适配 H7

位置：`application/other/remote_control.c` + `remote_control.h`

核心变更：

- **移除 F4 特有依赖**：
  - ~~`#include "bsp_usart.h"`~~ — F4 板载 USART 驱动，H7 用 `drv_uart`
  - ~~`#include "detect_task.h"`~~ — F4 掉线检测任务，改用 `HAL_GetTick()` 时间戳
  - ~~`#include "communication.h"`~~ — F4 多板通信，本板暂不需要
  - ~~`detect_hook(DBUS_TOE)`~~ — 改为内部时间戳更新
  - ~~`usart1_tx_dma_enable()`~~ — 移除 USART1 转发（本板不需要）
- **移除 `USART3_IRQHandler()`**：
  - 旧代码在中断里手动操作 DMA 双缓冲 + CT bit 切换
  - 新架构由 `drv_uart` 的 `HAL_UARTEx_RxEventCallback()` 处理，自动切换双缓冲并调用回调
- **新增 `remote_control_rx_callback()`**：
  - 接收回调，由 bsp_rc → drv_uart 在 DMA 空闲中断中触发
  - 根据帧长分发：18B → DT7 DBUS / 25B → SBUS (AT9S/HT8A/ET08A)
  - 同时填充 `i6x_ctrl_t` 数据（对 25B SBUS 帧调用 `sbus_to_i6x()`），保持兼容
- **`remote_control_init()` 简化**：
  - 旧：手动调用 `RC_Init(buf0, buf1, size)` 设置 DMA 双缓冲区
  - 新：调用 `RC_Init(remote_control_rx_callback)` 一行完成

#### 3.3 Task 层收口

位置：`User_File/4_Task/tsk_config_and_callback.cpp`

核心变更：

- 删除 `UART5_SBUS_Callback()` 局部回调
- 将 `UART_Init(&huart5, UART5_SBUS_Callback)` 替换为 `remote_control_init()`
- 新增 `#include "remote_control.h"`
- `RTOS_Remote_Task_Loop()` 中保留 `get_remote_control_point()` + `get_i6x_point()` 以便后续模式切换

### 4. 这次修改解决的问题

#### 问题 1：旧 bsp_rc 使用 F4 DMA 寄存器操作，在 H7 上不兼容

现象：直接操作 `DMA_SxCR_CT` / `USART3->DR` 等 F4 特有寄存器。
本次处理：改用 `drv_uart` 中间件的 `HAL_UARTEx_ReceiveToIdle_DMA()` 接口。

#### 问题 2：remote_control.c 引用了多个 F4 平台特有头文件/函数

现象：`bsp_usart.h` / `detect_task.h` / `communication.h` / `detect_hook()` / `usart1_tx_dma_enable()` 均在 H7 工程中不存在。
本次处理：全部移除，用 H7 工程已有的等价机制替代。

#### 问题 3：DJI DT7 接达妙 UART5 时停止位不匹配

现象：CubeMX 配置为 SBUS 的 2 stop bits，DT7 DBUS 实际是 1 stop bit，会导致帧错误。
本次处理：`RC_Init()` 根据 `__RC_TYPE` 在运行时修正停止位。

#### 问题 4：USART3_IRQHandler 中断处理不适用于 H7 的 UART5

现象：旧代码在 `USART3_IRQHandler` 中手动处理 DMA 双缓冲和空闲中断。
本次处理：完全删除，改由 `drv_uart` 中间件的全局 `HAL_UARTEx_RxEventCallback()` 统一处理。

### 5. 当前遗留问题

#### 问题 1：SBUS RX 反相依赖板载硬件电路

现状：达妙MC02 的 SBUS 接口已有硬件反相电路，软件层不做反相。
影响：如果使用其他开发板（无硬件反相器），需要手动在 `bsp_rc.cpp` 中取消注释启用软件 RX 反相。

#### 问题 2：`RTOS_Remote_Task_Loop()` 中遥控状态机尚未实现

现状：目前 20ms 周期任务只是占位，未实现模式切换。
后续怎么改：在 `RTOS_Remote_Task_Loop()` 中根据 `GetDt7RcSw()` / `GetRcOffline()` 实现状态机。

#### 问题 3：F4 备份文件需手动清理

现状：`bsp_rc_old_f4.c.bak` / `remote_control_old_f4.c.bak` 保留用于参考。
后续怎么改：确认新代码稳定后删除。

### 6. 数据流总览

```
[遥控器] ──RF──→ [接收机] ──UART5──→ [DMA + IDLE中断]
                                         │
                                    drv_uart.cpp
                                  HAL_UARTEx_RxEventCallback()
                                         │
                                    bsp_rc.cpp
                                  BSP_RC_UART5_Wrapper()
                                         │
                                  remote_control.c
                                remote_control_rx_callback()
                                    ┌────┴────┐
                               18B: DT7      25B: SBUS
                            sbus_to_rc()   Et08aSbusToRc() + sbus_to_i6x()
                                    │           │
                              RC_ctrl_t     i6x_ctrl_t
                                    │
                           Application Layer
                     GetDt7RcCh() / GetRcOffline() / ...
```

### 7. 后续建议

- 遥控模式切换可在 `RTOS_Remote_Task_Loop()` 中实现 FSM。
- 如需支持新的遥控器型号，在 `remote_control.c` 中新增对应的 SBUS 解析函数，并在 `robot_typedef.h` 中添加宏定义。
- 切换遥控器只需修改 `robot_param.h` 中的 `__RC_TYPE` 宏。

---

## 2026-04-06 IMU & USB 通信层适配达妙 MC02 (H7)

### 1. 本次修改目标

- 将 IMU 驱动和解算代码从大疆 C 板 (STM32F407) 适配到达妙 MC02 开发板 (STM32H723)。
- 移除所有不存在的 F407 依赖（bmi088driver、ist8310driver、bsp_spi、bsp_imu_pwm、ahrs、detect_task、kalman_filter、pid 等）。
- 使用达妙板级支持包 `BSP_BMI088` 的 C++ 类替代原始裸操作。
- 移除 IST8310 磁力计相关代码（达妙板无磁力计）。
- 将 USB 通信任务从 F407 USB CDC 直接操作重构为使用 `drv_usb.h` 中间件。
- 新增 CRC8/CRC16 校验模块（原项目缺失该依赖）。
- 移除不存在的 `supervisory_computer_cmd.h`，将其 API 合并到 `usb_task.h`。

### 2. 修改文件

#### 重写文件

| 文件                                                | 说明                                                     |
| --------------------------------------------------- | -------------------------------------------------------- |
| `application/IMU/IMU_task.h`                        | 移除所有 F407 DMA/IST8310/温控常量，简化为 3 个 API 声明 |
| `application/IMU/IMU_task.cpp`（原 `.c`）           | 完全重写，使用 BSP_BMI088 替代手动 SPI DMA 和 IST8310    |
| `application/IMU/IMU_solve.h`                       | 移除 kalman_filter.h 依赖和 INS_t 结构体                 |
| `application/IMU/IMU_solve.cpp`（原 `.c`）          | 简化为 BSP_BMI088 的薄封装                               |
| `application/communication/usb_task.cpp`（原 `.c`） | 使用 drv_usb 中间件，回调接收替代轮询                    |

#### 修改文件

| 文件                                    | 说明                                        |
| --------------------------------------- | ------------------------------------------- |
| `application/communication/usb_task.h`  | 添加 `extern "C"` 守卫，合入 GetScCmd* 声明 |
| `application/communication/usb_debug.h` | 添加 `extern "C"` 守卫                      |

#### 新增文件

| 文件                                     | 说明                              |
| ---------------------------------------- | --------------------------------- |
| `application/communication/CRC8_CRC16.h` | CRC8/CRC16 校验声明               |
| `application/communication/CRC8_CRC16.c` | CRC8(0x31)/CRC16(0x8005) 查表实现 |

### 3. 具体改了什么

#### 3.1 IMU 驱动层 (`IMU_task`)

**为什么改：**
- 原代码直接操作 SPI1 DMA 寄存器（F407），达妙板使用 SPI2，且 BSP 层已封装。
- 原代码包含 IST8310 磁力计初始化和数据读取，达妙板无磁力计。
- 原代码手动管理 BMI088 加速度计/陀螺仪的 SPI CS 和 DMA 回调。

**改成了什么：**
- `IMU_task.cpp` 直接使用 `BSP_BMI088` 对象（在 `tsk_config_and_callback.cpp` 中实例化）的 API：
  - `BSP_BMI088.Get_Euler_Angle()` → 欧拉角
  - `BSP_BMI088.Get_Gyro_Body()` → 机体角速度
  - `BSP_BMI088.Get_Accel()` → 加速度
- 移除了 ~450 行 F407 特有代码（DMA 回调、SPI 命令构建、IST8310 驱动、温度 PID、板旋转矩阵）。
- IMU 数据通过 `Publish/Subscribe` 机制发布。

#### 3.2 IMU 解算层 (`IMU_solve`)

**为什么改：**
- 原代码依赖 `kalman_filter.h`（F407 项目文件，本项目不存在）。
- 原 EKF 解算约 400 行，BSP_BMI088 已内置 `Class_Filter_EKF<4,3,3>` 完成姿态融合。

**改成了什么：**
- `GetEkfAngle(axis)` → 转发到 `BSP_BMI088.Get_Euler_Angle()`。
- `GetEkfAccel(axis)` → 转发到 `BSP_BMI088.Get_Accel()`。
- 代码从 ~468 行缩减到 ~50 行。

#### 3.3 USB 通信层 (`usb_task`)

**为什么改：**
- 原代码使用 `USB_Transmit()`/`USB_Receive()`（F407 USB CDC，本项目未定义）。
- 原代码调用 `MX_USB_DEVICE_Init()`（由 `Task_Init` 中的 `USB_Init()` 统一初始化）。
- 原代码包含 `supervisory_computer_cmd.h`（本项目不存在）。

**改成了什么：**
- 发送：`USB_Transmit()` → `USB_Transmit_Data()`（来自 `drv_usb.h`）。
- 接收：轮询 `USB_Receive()` → 回调 `UsbRxCallback()`（通过 `USB_Init()` 注册）。
- 将 `GetScCmd*` 系列函数声明合并到 `usb_task.h`。
- 保留了完整的 CRC8 帧头校验 + CRC16 全帧校验逻辑。
- 保留了 13 种发送数据包 + 3 种接收数据包的协议结构。

#### 3.4 CRC8/CRC16 模块

**为什么新增：**
- USB 协议帧需要 CRC8 校验帧头和 CRC16 校验全帧。
- 原项目中该模块缺失。

**实现：**
- CRC8: 多项式 0x31，初始值 0xFF，256 项查找表。
- CRC16: 多项式 0x8005，初始值 0xFFFF，256 项查找表。

### 4. 架构对比

```
【原 F407 架构】                          【新 H7 架构】
IMU_task.c                                IMU_task.cpp
  ├── SPI1 DMA 手动操作                     └── BSP_BMI088.Get_Euler_Angle()
  ├── IST8310 磁力计驱动                         BSP_BMI088.Get_Gyro_Body()
  ├── BMI088 裸寄存器读取                         BSP_BMI088.Get_Accel()
  ├── 温度 PID 控制
  └── HAL_GPIO_EXTI_Callback

IMU_solve.c                               IMU_solve.cpp
  ├── 自定义 EKF (kalman_filter.h)          └── BSP_BMI088 内置 EKF 封装
  ├── 四元数手动计算
  └── 重力向量估计

usb_task.c                                usb_task.cpp
  ├── USB_Receive() 轮询                    ├── UsbRxCallback() 回调
  ├── USB_Transmit() 直接发送                ├── USB_Transmit_Data() 中间件
  ├── MX_USB_DEVICE_Init()                  └── USB_Init() 统一初始化
  └── supervisory_computer_cmd.h
```

### 5. 后续使用说明

#### IMU 数据获取

```c
#include "IMU.h"

// 获取欧拉角 (rad)
float yaw   = GetImuAngle(AX_Z);
float pitch = GetImuAngle(AX_Y);
float roll  = GetImuAngle(AX_X);

// 获取角速度 (rad/s)
float gyro_yaw = GetImuVelocity(AX_Z);

// 获取加速度 (m/s²)
float accel_x = GetImuAccel(AX_X);
```

#### USB 通信

```c
#include "usb_task.h"
#include "usb_debug.h"

// 发送调试数据（在任意位置调用）
ModifyDebugDataPackage(0, some_value, "target");

// 获取上位机指令
float cmd_yaw = GetScCmdGimbalAngle(AX_YAW);
bool fire = GetScCmdFire();
```

#### 注意事项

1. **USB 回调冲突**：`usb_task` 启动时会通过 `USB_Init()` 重新注册接收回调，覆盖 `tsk_config_and_callback.cpp` 中为 Vofa 设置的回调。如需同时使用 Vofa 调试和 USB 协议通信，需自行实现回调分发。
2. **usb_task 未自动创建**：当前 `freertos.c` 中未创建 usb_task 线程。如需启用，请在 `MX_FREERTOS_Init()` 中添加任务创建。
3. **BSP_BMI088 初始化**：IMU 硬件初始化由 `tsk_config_and_callback.cpp` 中的 `Task_Init()` 完成，IMU_task 只负责读取数据。

### 6. 当前已知问题 / 限制

1. `usb_task` 线程未在 FreeRTOS 中创建（需用户手动添加）。
2. USB 接收回调会覆盖 Vofa 回调，两者不能同时使用。
3. `usb_typdef.h` 中的协议结构未修改（协议层与硬件无关，保持原样）。
4. **编译阻断（预存问题）**：`application/chassis/chassis_balance.h` 包含 `kalman_filter.h`、`motor.h`、`pid.h`、`user_lib.h` 等 F407 旧依赖，这些文件在达妙项目中不存在。由于 `robot_param_balanced_infantry.h` 设置了 `CHASSIS_TYPE CHASSIS_BALANCE`，导致这些 `#include` 被激活。此错误与本次 IMU/USB 重构无关，需后续单独重构底盘模块（用 `alg_filter_kalman.h`、`alg_pid.h` 等中间件替代）。
5. **预存警告**：`alg_basic.cpp`（符号比较）、`dvc_motor_dm.cpp`（switch 未覆盖）、`bsp_ws2812.cpp`（未使用变量）存在预存警告，与本次修改无关。

---

## 2026-04-07 底盘平衡控制全栈适配达妙 MC02 (H7)

### 1. 本次修改目标

- 将 `application/chassis/` 目录下所有文件从大疆 C 板 (STM32F407) 完整适配到达妙 MC02 (STM32H723)。
- 移除所有不存在的 F407 依赖（`kalman_filter.h`、`motor.h`、`pid.h`、`user_lib.h`、`CAN_communication.h`、`bsp_delay.h`、`detect_task.h`、`signal_generator.h` 等）。
- 使用达妙中间件 C++ 类替代原始 F407 驱动：`Class_Motor_DM_Normal`、`Class_PID`、`Class_Filter_Kalman<2,1,2>`。
- 修复跨文件 `extern inline` 警告。
- 达成 **0 错误 0 警告** 编译。

### 2. 修改文件

#### 重写文件

| 文件                                                 | 说明                                                      |
| ---------------------------------------------------- | --------------------------------------------------------- |
| `application/chassis/chassis_balance.h`              | 完全重写，所有类型从 F407 裸结构体迁移到达妙中间件 C++ 类 |
| `application/chassis/chassis_balance.cpp`（原 `.c`） | 完全重写 ~800 行，适配新 Motor/PID/Kalman/CAN API         |

#### 修改文件

| 文件                                           | 说明                                                    |
| ---------------------------------------------- | ------------------------------------------------------- |
| `application/chassis/chassis_balance_extras.h` | 添加 `extern "C"` 守卫，修复 `extern inline` → `extern` |
| `application/chassis/chassis_balance_extras.c` | 移除 `inline` 关键字（`CalcLegLengthDiff`）             |
| `application/chassis/chassis.h`                | 添加 `extern "C"` 守卫                                  |
| `application/IMU/IMU.h`                        | `extern inline` → `extern`（3 个函数）                  |
| `application/IMU/IMU_task.cpp`                 | 移除定义处 `inline`（3 个函数）                         |
| `application/other/remote_control.h`           | `extern inline` → `extern`（6 个函数）                  |
| `application/other/remote_control.c`           | 移除定义处 `inline`（6 个函数）                         |
| `application/communication/usb_task.cpp`       | `Subscribe()` 参数添加 `(char *)` 强转                  |

#### 删除文件

| 文件                                    | 说明             |
| --------------------------------------- | ---------------- |
| `application/chassis/chassis_balance.c` | 已被 `.cpp` 替代 |

### 3. 具体改了什么

#### 3.1 chassis_balance.h — 类型系统迁移

**移除的 F407 依赖：**

```c
// 以下头文件全部移除
#include "kalman_filter.h"   // F407 卡尔曼滤波
#include "motor.h"           // F407 通用电机驱动
#include "pid.h"             // F407 PID 控制器
#include "user_lib.h"        // F407 工具库
#include "CAN_communication.h"
#include "bsp_delay.h"
#include "detect_task.h"
```

**替代的达妙中间件头文件：**

```cpp
#include "dvc_motor_dm.h"     // Class_Motor_DM_Normal (DM8009 MIT模式)
#include "alg_pid.h"          // Class_PID
#include "alg_filter_kalman.h"// Class_Filter_Kalman<State, Input, Measurement>
#include "drv_can.h"          // CAN_Transmit_Data (LK电机原始帧)
#include "alg_basic.h"        // Basic_Math_Constrain / Basic_Math_Abs
```

**核心类型迁移表：**

| F407 原类型                 | 达妙新类型                   | 用途                  |
| --------------------------- | ---------------------------- | --------------------- |
| `Motor_s` (自定义)          | `Class_Motor_DM_Normal`      | DM8009 关节电机 (4个) |
| `pid_type_def`              | `Class_PID`                  | 所有 PID 控制回路     |
| `KalmanFilter_t`            | `Class_Filter_Kalman<2,1,2>` | 观测器卡尔曼滤波      |
| `first_order_filter_type_t` | `LowPassFilter_t` (本地定义) | 一阶低通滤波          |
| `Motor_s` (LK 轮毂)         | `WheelMotor_t` (本地定义)    | MF9025 轮毂电机 (2个) |

**新增本地类型：**

- `LowPassFilter_t`：简单 alpha-IIR 低通滤波器（达妙中间件只有 FIR `Class_Filter_Frequency`，不适用于实时控制）
- `WheelMotor_t`：LK/瓴控 MF9025 轮毂电机反馈/设定值结构（达妙中间件无 LK 电机驱动）

#### 3.2 chassis_balance.cpp — 控制逻辑适配

**（原 chassis_balance.c → .cpp，C → C++ 重命名）**

**电机 API 迁移：**

```
F407:  MotorInit(&motor, id, can, type, dir, ...)
H7:    motor.Init(&hfdcanN, rx_id, tx_id,
                  Motor_DM_Control_Method_NORMAL_MIT,
                  12.5f, 25.0f, 10.0f)

F407:  GetMotorMeasure(&motor) → motor.measure.angle/velocity/torque
H7:    motor.Get_Now_Angle() / Get_Now_Omega() / Get_Now_Torque()
       (CAN 回调自动更新，无需手动调用)

F407:  DmMitCtrlTorque(hcan, id, torque)
H7:    motor.Set_Control_Angle(0); motor.Set_Control_Omega(0);
       motor.Set_K_P(0); motor.Set_K_D(0);
       motor.Set_Control_Torque(torque);
       motor.TIM_Send_PeriodElapsedCallback()

F407:  DmEnable(hcan, id)  /  DmMitStop(hcan, id)
H7:    motor.CAN_Send_Enter()  /  motor.CAN_Send_Exit()
```

**PID API 迁移：**

```
F407:  PID_init(&pid, mode, params[], max_out, max_iout)
H7:    pid.Init(kp, ki, kd, 0, max_iout, max_out, 0.001f)

F407:  PID_calc(&pid, now, target)  → 返回 pid.out
H7:    pid.Set_Now(now); pid.Set_Target(target);
       pid.TIM_Calculate_PeriodElapsedCallback();
       pid.Get_Out()

F407:  PID_clear(&pid)
H7:    pid.Set_Integral_Error(0)
```

**卡尔曼滤波 API 迁移：**

```
F407:  Kalman_Filter_Init(&kf, n, 0, m)
       kf.FilteredValue[i] / kf.MeasuredVector[i] / kf.A_Data[i]
H7:    kf.Init(A, B, H, Q, R, P, X, U)
       kf.Vector_X.Data[i] / kf.Vector_Z.Data[i] / kf.Matrix_A.Data[i]
       kf.TIM_Predict_PeriodElapsedCallback()
       kf.TIM_Update_PeriodElapsedCallback()
```

**LK 轮毂电机（无达妙驱动，自研 CAN 帧）：**

```cpp
// 双电机扭矩控制 (CAN ID 0x280)
static void LkMultipleTorqueControl(FDCAN_HandleTypeDef *hcan,
                                     int16_t torque1, int16_t torque2) {
    uint8_t data[8] = { ... };  // [0:1]=torque1, [2:3]=torque2, [4:7]=0
    CAN_Transmit_Data(hcan, 0x280, data, 8);
}
```

**DWT 延时（替代 bsp_delay.h）：**

```cpp
static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}
```

**GIMBAL_TYPE 兼容：**

当 `GIMBAL_TYPE == GIMBAL_NONE` 时，`GetGimbalDeltaYawMid()` 和 `GetGimbalInitJudgeReturn()` 缺少定义。在 `.cpp` 中提供静态 stub 实现：

```cpp
#if GIMBAL_TYPE == GIMBAL_NONE
static float GetGimbalDeltaYawMid(void) { return 0.0f; }
static int GetGimbalInitJudgeReturn(void) { return 0; }
#endif
```

#### 3.3 extern inline 修复

**问题根因：**

F407 项目中 `extern inline` 在纯 C 编译时遵循 GNU inline 语义（外部可见 + 编译器可选择内联），但当对应 `.h` 被 C++ 编译单元（`.cpp`）include 时，C++ 标准 `inline` 语义不同，GCC 会产生 "inline function used but never defined" 警告。

**修复范围：**

| 头文件                     | 函数                                                                                            |
| -------------------------- | ----------------------------------------------------------------------------------------------- |
| `IMU.h`                    | `GetImuAngle`, `GetImuVelocity`, `GetImuAccel`                                                  |
| `remote_control.h`         | `GetRcOffline`, `GetDt7RcCh`, `GetDt7RcSw`, `GetDt7MouseSpeed`, `GetDt7Mouse`, `GetDt7Keyboard` |
| `chassis_balance_extras.h` | `CalcLegLengthDiff`                                                                             |

**修复方式：** 声明处移除 `inline`，定义处移除 `inline`，保留普通 `extern` 函数链接。

### 4. 架构对比

```
【原 F407 架构】                          【新 H7 架构】
chassis_balance.c                         chassis_balance.cpp
  ├── Motor_s (自定义裸结构体)               ├── Class_Motor_DM_Normal ×4 (关节)
  ├── pid_type_def (F407 PID)                ├── WheelMotor_t ×2 (轮毂, 自研CAN)
  ├── KalmanFilter_t (F407 卡尔曼)           ├── Class_PID ×N (中间件PID)
  ├── first_order_filter_type_t              ├── Class_Filter_Kalman<2,1,2> ×N
  ├── DmMitCtrl*() (裸CAN发送)              ├── LowPassFilter_t (本地简单IIR)
  ├── LkMultiple*() (裸CAN发送)             ├── motor.TIM_Send_PeriodElapsedCallback()
  ├── PID_calc/init/clear()                  ├── pid.TIM_Calculate_PeriodElapsedCallback()
  ├── Kalman_Filter_Init/Update()            ├── kf.TIM_Predict/Update_...Callback()
  ├── bsp_delay.h → delay_us()              └── DWT->CYCCNT busy-wait delay_us()
  └── detect_task.h → toe_is_error()
```

### 5. 业务逻辑保留情况

以下控制逻辑**完整保留**，仅替换底层 API 调用：

| 模块             | 说明                                                      |
| ---------------- | --------------------------------------------------------- |
| LQR 状态反馈     | 12 维状态向量 × 增益矩阵，输出扭矩/力                     |
| VMC 虚拟模型控制 | 五连杆正/逆运动学，腿部扭矩→关节扭矩映射                  |
| 卡尔曼观测器     | 机体速度/位移估计 (2 状态 × 2 观测)                       |
| 抬腿检测         | 腿长差值+支撑力判断，PD 回中策略                          |
| 防劈叉           | 左右腿长差异 PID 补偿                                     |
| 台阶检测         | 虚拟腿长突变分析                                          |
| 多模式控制       | ZeroForce/Calibrate/OffHook/Normal/Debug/PosDebug/StandUp |
| 校准流程         | 关节电机零位标定（MIT 力矩→位置锁定→保存零点）            |

### 6. 当前已知问题 / 限制

1. **LK MF9025 轮毂电机无中间件驱动**：当前使用自研 `LkMultipleTorqueControl()` / `LkMultipleIqControl()` 通过原始 CAN 帧控制。后续如达妙中间件新增 LK 驱动，可替换。
2. **底盘任务未启用**：`tsk_config_and_callback.cpp` 中 `App_Chassis_Init()`、`App_Chassis_Ctrl_Task_Loop()`、`App_Chassis_Monitor_Task_Loop()` 调用仍被注释。启用前需确认 CAN ID 分配和电机接线。
3. **RAM_D1 占用 97.75%**：接近满载，新增大型全局变量时需注意。
4. **GIMBAL_TYPE 默认 GIMBAL_NONE**：云台联动功能（yaw 补偿、初始化判断）使用 stub 返回 0。启用云台后需修改 `robot_param.h`。
5. **备份文件待清理**：`chassis_balance.c.bak`、`chassis_balance.h.bak` 保留用于参考，确认稳定后可删除。

### 7. 编译结果

```
Build: 0 errors, 0 warnings
FLASH: ~166 KB / 1 MB (15.8%)
RAM_D1: ~320 KB / 320 KB (97.75%)
```
