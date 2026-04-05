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
