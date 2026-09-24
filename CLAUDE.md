# CLAUDE.md — 机器人嵌入式代码架构规范

> 本文件每次会话自动加载。**动手改任何代码之前，先读完本文件。**
> 对应文档：《机器人嵌入式代码架构规范与修改日志》 V1.0，生效日期 2026-09-16。
> 适用范围：STM32G491VETx 底盘工程、下位机运动控制、位姿融合模块。

---

## 0. 先读这一条：本规范是【目标架构】，尚未落地

本规范描述的是**目标架构**。当前仓库里**大部分内容还不存在**。不要假装它们已经存在，
也不要为了"符合规范"去大规模重构现有代码——那属于破坏性改动，需要用户明确授权。

现状对照表见第 2 节。**新增代码按本规范写；现有代码保持不动，除非用户明确要求迁移。**

---

## 1. 四层架构与修改权限

```
【应用层 Task】→【业务层 Module】→【抽象设备层 Device】→【硬件驱动层 Driver】
```

| 层级 | 核心职责 | 允许修改 | 严格禁止 |
|---|---|---|---|
| 应用层 | 状态机调度、任务流程、比赛策略 | 新增任务、调整状态跳转 | 修改对业务层的接口调用格式 |
| 业务层 | 底盘控制、位姿融合、运动规划、PID | 优化算法内部逻辑、新增融合算法 | 修改抽象接口调用方式、修改核心数据结构 |
| 抽象设备层 | 统一接口定义、数据结构标准 | 扩展接口字段（兼容原有格式） | 删除/修改已有函数指针、修改结构体字段顺序 |
| 硬件驱动层 | 传感器解析、外设驱动、寄存器操作 | 新增驱动、替换驱动实现、修复硬件 bug | 改动向上输出的接口格式 |

**核心约束：面向接口编程，驱动与业务完全解耦；切换数据源只改驱动层，上层业务零改动。**

---

## 2. 现状与目标架构的差距（重要）

当前仓库结构：

```
app/              应用层：任务调度（banyuntask、worker_task）、比赛策略（ColorIdentif）
                  ＋ 尚待下沉的业务模块（NavigationMecanum、BollLocator、Mecanum_Move）
algorithm/        业务层：mecanum、pid、angle_ctrl、Circle_base、Nav_position
device/           抽象设备层：PoseData_t + LocatorDev_t 接口 ＋ locator_wheel 实现（OPS9/光流尚未接入）
hardware/         硬件驱动层，按器件类型细分：
  ├─ sensors/       hwt_imu、grayscale、color、collect_ir、k230、QRcode
  ├─ actuators/     emm_v5、Send_motor、block_basic、servo_scs（＋scslib/ 厂商舵机库，见 V1.10.0）
  ├─ display/       oled、oled_data
  ├─ bus/           sw_uart、uart2_tbop10
  └─ Common_used.h  工程公共头（只聚合 libc + HAL + FreeRTOS）
debug/            调试工具目录（当前为空 —— 原在线调参工具 trace_tune 已随循迹功能删除）
config/           参数配置目录（已建，param_config.h 待第 7.3 节落地）
clauderecord/     变更日志归档，按日期分文件（见第 6 节）；纯 Markdown，不参与编译
Core/             CubeMX 生成（main.c、app_freertos.c、外设初始化）
uart/             msp_uart2.c
obsolete/         已停用的驱动（imu660/、hwt101_legacy/、wit_protocol/），不在任何 CMake glob 内，不参与编译
```

> V1.3.0 做了一轮分层归位（`pid` / `angle_ctrl` / `Trace_base` / `Circle_base` 迁入 `algorithm/`，
> `wit_protocol` 移入 `obsolete/`）。V1.5.0 做了第二轮：`hardware/` 按器件类型细分，
> `ColorIdentif` 归入 `app/`，`trace_tune` 移入新的 `debug/`，并**拆掉了 `Common_used.h`
> 这一聚合头** —— 它此前使 include 图退化成完全图，是第 1 节解耦要求失效的根因。
>
> V1.4.0 接入了任务调度层（`app/worker_task.c` 的 `FC_TASK` / `NLF_TASK` +
> `Core/Src/app_freertos.c` 的调度器）。此前全工程只有一个空循环任务，应用代码
> 被 `--gc-sections` 整段回收、根本没进 `.elf`。**但流程入口与驱动源仍空**：
> `task_send()` 零调用，`NLF_RunFlow()` 只写死了 Mode → 执行体的映射。
>
> V1.6.0 **移除了整个循迹功能**（`GrayTrace` / `Trace_base` / `trace_tune` 共 6 个文件），
> 只保留 K230 找圆与导航。V1.5.0 遗留的 ② `trace_tune` 双向耦合、③ `QRcode.c` 中断
> 回调硬编码分流，随之消失。
>
> **仍待处理**（按优先级）：① `app/` 内三个业务模块向 `algorithm/` 的下沉（需先定义
> 位姿输入 / 底盘输出 / 阻塞旋转 / 计时四个注入式接口）；② `config/param_config.h`
> 的参数收拢；③ 确认 `grayscale.c` / `k230.c` 里失去调用者的驱动 API 是否还要保留。

| 规范中的名字 | 仓库现状 | 说明 |
|---|---|---|
| `drivers/` 目录 | **不可用** | Windows 大小写不敏感，与厂商目录 `Drivers/`（STM32 HAL）冲突。改用 `device/` |
| `PoseData_t` | 不存在 | 现有位姿结构是 `app/NavigationMecanum.h:21` 的 `World_Dir_t`，只有 `x / y / yaw` |
| `LocatorDev_t` | 不存在 | 本次在 `device/locator_dev.h` 建立骨架 |
| `active_locator` | 不存在 | 阶段 1 建应用任务时引入 |
| `locator_wheel` | **已实现** | `device/drv_wheel_odom.c`，代码由 `Nav_position.c` 迁入 |
| `locator_ops9` / `locator_optical_flow` | 均不存在 | 硬件未接入 |
| `drv_wheel_odom.c` | **已存在** | `algorithm/Nav_position.c` 已降级为兼容适配层（旧接口转调 locator_wheel） |
| `drv_ops9.c` | 不存在 | 无 OPS9 硬件接入 |
| `drv_optical_flow.c` | 不存在 | 无光流硬件接入 |
| `modules/pose_fusion.c` | 不存在 | 无独立融合模块 |
| `config/param_config.h` | 不存在 | 现有可调参数散落在各模块头文件中 |
| 姿态来源 | `hardware/hwt_imu.c` | 维特 HWT906，I2C3，航向取 `g_hwt_imu_yaw_rad` |

**现有可用定位源实际上只有轮式里程计一种**，航向由 HWT906 提供（见 `Nav_position.c`）。

> 注意：`algorithm/Nav_position.h` 中的 INS 相关代码已删除，原因是 HWT906 只输出欧拉角、
> 无原始加速度数据源。原实现保留在 `obsolete/imu660/Nav_position_INS_reference.c`。

### 2.1 定位源契约（设计决策，勿改）

**每个定位设备对上层承诺的输出只有三个量：世界系 `x` / `y` / `yaw`。**
上层不关心这三个量是怎么来的。

`locator_wheel` 当前这样满足契约：

| 量 | 来源 |
|---|---|
| `x`、`y` | 四轮编码器增量推算（相对里程） |
| `yaw` | `hardware/hwt_imu.c` 的 `g_hwt_imu_yaw_rad`（HWT906 实测航向） |

**在轮式里程计驱动里读 IMU 是【有意设计】，不是待还的技术债。**
轮式里程计本身给不出绝对航向，所以必须借 HWT906；而后继的 `locator_ops9`
会直接输出 x/y/yaw。两者对上层是同一个契约，业务层切换定位源时只换
`LocatorDev_t` 的实现，不需要知道 yaw 从哪来。

> **不要把 IMU 读取从 `drv_wheel_odom.c` 里拆出去。** 拆了会让 `locator_wheel`
> 变成"只给 x/y 的半成品"，反而破坏上层契约的统一性。

---

## 3. 核心数据结构 `PoseData_t`（强约束）

定义位置：`device/pose_data.h`

所有定位传感器、融合算法必须统一使用该结构体输出，**禁止私自定义位姿格式**。

```c
typedef struct {
    float x;          // 全局X坐标，单位：m
    float y;          // 全局Y坐标，单位：m
    float yaw;        // 航向角，单位：rad
    float pitch;      // 俯仰角，单位：rad
    float roll;       // 横滚角，单位：rad

    float vx;         // X轴线速度，单位：m/s
    float vy;         // Y轴线速度，单位：m/s
    float wz;         // 航向角速度，单位：rad/s

    uint8_t  valid;     // 数据有效性标志：0-无效，1-有效
    uint32_t timestamp; // 数据时间戳，单位：ms
} PoseData_t;
```

使用规则：

1. 驱动层解析硬件原始数据后，必须填充为 `PoseData_t` 再向上输出；
2. 业务层只能通过 `get_pose` 接口读取，禁止直接访问驱动层内部变量；
3. 新增字段只能追加到末尾，保证前向兼容。

### 坐标系约定（沿用现有工程约定）

- X 轴：前方（车头朝向 0° 时正对的方向）
- Y 轴：左方（右手系，Z 轴向上）
- yaw：世界航向角，弧度制，0 = 正对 X 轴，CCW 为正，范围 [-π, π]

---

## 4. 抽象设备接口 `LocatorDev_t`（强约束）

定义位置：`device/locator_dev.h`

所有定位传感器（轮式里程计 / OPS9 / 光流 / 视觉定位）必须实现该接口：

```c
typedef struct {
    void    (*init)(void);                    // 设备初始化：外设初始化、协议初始化
    void    (*update)(void);                  // 周期更新：解析数据、更新缓存、校验有效性
    void    (*get_pose)(PoseData_t *pose_out);// 获取最新位姿到 pose_out
    uint8_t (*is_healthy)(void);              // 健康检查：在线 & 数据可信
} LocatorDev_t;
```

已注册设备实例（目标状态，尚未全部实现）：

| 实例名 | 对应传感器 | 实现文件 |
|---|---|---|
| `locator_wheel` | 轮式里程计 | `device/drv_wheel_odom.c` |
| `locator_ops9` | OPS9 光学定位模块 | `device/drv_ops9.c` |
| `locator_optical_flow` | 光流传感器 | `device/drv_optical_flow.c` |

业务层标准调用方式（**切换传感器只改这一行指针**）：

```c
const LocatorDev_t *active_locator = &locator_ops9;   // ← 唯一需要改的地方

active_locator->init();
// 周期循环：
active_locator->update();
active_locator->get_pose(&robot_pose);
```

---

## 5. AI 修改规则与操作边界

### 5.1 允许

1. 驱动层：新增/替换/修复定位传感器驱动，实现 `LocatorDev_t` 接口；
2. 算法层：优化位姿融合算法、PID 参数、滤波参数，新增融合策略；
3. 配置层：修改可调参数宏定义、常量；
4. 应用层：新增任务状态、调整任务调度逻辑。

### 5.2 严格禁止

1. **禁止修改 `PoseData_t` 已有字段的顺序、类型**；
2. **禁止修改 `LocatorDev_t` 接口的函数指针定义、参数格式**；
3. **禁止业务层直接调用驱动层内部函数，必须通过抽象接口**；
4. **禁止破坏四层架构、跨层直接操作硬件寄存器**。

### 5.3 修改优先级

1. 优先改配置参数，不改逻辑代码；
2. 优先新增驱动文件，不改已有驱动；
3. 优先扩展接口，不删除原有接口。

---

## 6. 修改记录管理规范

### 6.1 版本号规则（语义化版本 主.次.修订）

- **主版本**：架构重构、接口不兼容升级
- **次版本**：新增功能、新增模块、接口兼容扩展
- **修订号**：bug 修复、参数优化、驱动微调


### 6.2 记录写在哪里

**完整记录一律写入 `clauderecord/YYYY-MM-DD.md`（当天日期）。不再往本文件里追加正文。**

1. 当天文件已存在 → 追加到文件末尾（同日内按版本号升序排列）；
2. 当天文件不存在 → 新建，表头照抄 `clauderecord/` 里任一已有文件（`# 变更记录 · 日期`
   + 两行 `>` 说明 + 空行）；
3. 无论哪种情况，都要在 `## 变更日志` 的索引表里**加一行**（版本 / 日期 / 摘要 / 记录文件）。
   本文件只保留索引，不保留正文 —— 它的每次会话都会被完整读进上下文，正文留在里面
   会让上下文被历史淹没（这正是 V1.9.0 拆分的原因）。

必填字段：版本号 / 修改日期 / 修改类型 / 涉及模块 / 修改内容 / 影响范围 / 验证状态 / 备注。

填写要求：

1. 每次修改**至少一条**记录，禁止多条修改合并为一条；
2. 必须明确修改类型与影响范围，禁止模糊描述；
3. 接口变更必须标注兼容性：**兼容升级 / 不兼容升级**。

模板：

```markdown
#### V1.0.1 - YYYY-MM-DD
- **修改类型**：新增 / 修改 / 删除 / 优化 / 修复
- **涉及模块**：层级 / 文件名
- **修改内容**：改了什么、为什么改
- **影响范围**：可能影响的功能模块
- **验证状态**：未验证 / 已验证 / 待测试
- **备注**：补充说明
```

---

## 7. 扩展开发指南

### 7.1 新增定位传感器

1. 新建 `device/drv_xxx.c/h`（**不是 `drivers/`**，原因见第 2 节）；
2. 实现 `LocatorDev_t` 的四个标准函数；
3. 注册实例 `locator_xxx`；
4. 修改 `active_locator` 指针指向新实例，验证功能；
5. 追加修改记录（`clauderecord/YYYY-MM-DD.md`，见第 6.2 节）。

### 7.2 新增位姿融合算法

1. 输入多个 `PoseData_t`，输出融合后的 `PoseData_t`；
2. **禁止修改单个驱动的实现**，融合逻辑独立封装；
3. 支持按时间戳对齐、动态权重调整；
4. 追加修改记录（`clauderecord/YYYY-MM-DD.md`，见第 6.2 节）。

### 7.3 参数配置

1. 所有可调参数统一放 `config/param_config.h`（**待建**，见第 2 节）；
2. 使用宏定义并注释含义、取值范围；
3. **禁止在业务逻辑代码中硬编码魔法数字**；
4. 修改参数同样需要记录到变更日志（`clauderecord/YYYY-MM-DD.md`，见第 6.2 节）。

---

## 8. 构建说明（容易被忽略）

源码收集用 `file(GLOB_RECURSE ... CONFIGURE_DEPENDS)`（V1.5.0 起，此前是 `GLOB`），
顶层目录为：`Core/Src`、`app`、`algorithm`、`hardware`、`device`、`debug`、`config`。

- **在已有顶层目录下再建子目录**（如在 `hardware/` 下加 `imu/`）：**无需改 CMake**，
  `GLOB_RECURSE` 会自动收集；但若新子目录里的 `.c` 要 `#include` 同目录的头，
  仍需把该子目录加进 `target_include_directories`（include 路径没有递归形式）。
- **新建顶层层目录**：必须同时改两处，否则新代码不参与编译 ——
  1. `CMakeLists.txt` 的 `APP_SOURCES` glob 列表；
  2. `CMakeLists.txt` 的 `target_include_directories` 列表。

工程必须用 ARM 交叉工具链编译（`cmake --preset Debug`），用宿主 MinGW 编译会在
`CMakeLists.txt:25` 直接 `FATAL_ERROR`。

> **查告警一律用 `cmake --build --preset Debug --clean-first`。**
> 增量构建只重编改动过的文件，会漏掉本次改动引入的告警 —— V1.5.0 就因此漏检了
> 两处 `-Wimplicit-function-declaration`（隐式声明会被静默按 `int` 处理返回值，
> 只报 warning 不报 error）。

> **验证重构有没有改变行为，标准做法是比对目标文件而非 `.elf`。**
> `.elf` 会被 `-Wl,--gc-sections` 裁剪，未被引用的模块根本不在其中（见 V1.3.0 备注），
> 大小相同可能只是因为双方都没被链接。可靠做法：两次构建分别对全部 `.obj` 执行
> `objcopy --strip-debug` 后逐字节比对（剥调试信息是因为搬动源文件会改变 DWARF 里的路径）。

---

## 变更日志

**完整记录见 `clauderecord/`（按记录日期分文件）；本文件只保留索引，不再放正文。**
新增记录按第 6.2 节的步骤写入当天文件，并在下表加一行。

| 版本 | 日期 | 摘要 | 记录 |
|---|---|---|---|
| V1.10.7 | 2026-09-23 | 修复：SCS0009 总线字节序错误（`setEnd` 全局量按系列切换）；修正中位常量 | [2026-09-23](clauderecord/2026-09-23.md) |
| V1.10.6 | 2026-09-22 | 优化：精简 `gripper_task` 调试脚手架；建立 STS3032+SCS0009 双系列 6 舵机骨架 | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.5 | 2026-09-22 | 新增：广播动作验证发送通路（调试用，会让舵机动作） | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.4 | 2026-09-22 | 新增：PD2 悬空/被驱动判别探针；修复环回超时判据恒假 | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.3 | 2026-09-22 | 修复：`ftUart_Send` 去掉每帧 TE 翻转，与厂商实现对齐 | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.2 | 2026-09-22 | 新增：`gripper_task` 接舵机总线自检；修复栈溢出检查失效、栈/堆不足 | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.1 | 2026-09-22 | 新增：按 STS3032 补 EPROM 解锁对与回读包装（20→35 个） | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.0 | 2026-09-22 | 新增：飞特 SCS 总线舵机驱动包移植（UART5 / 1M） | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.9.0 | 2026-09-21 | 优化：变更日志拆分为 `clauderecord/`，本文件只留索引 | [2026-09-21](clauderecord/2026-09-21.md) |
| V1.6.1 | 2026-09-20 | 修复：CAN 从未启动导致「电机不转」 | [2026-09-20](clauderecord/2026-09-20.md) |
| V1.6.2 | 2026-09-20 | 修复：排查 HardFault，加固启动与任务栈 | [2026-09-20](clauderecord/2026-09-20.md) |
| V1.6.3 | 2026-09-20 | 修复：CAN 总线状态诊断 + bus-off 检测掩码错误 | [2026-09-20](clauderecord/2026-09-20.md) |
| V1.8.2 | 2026-09-20 | 修复：printf 重定向失效（重定向点用错） | [2026-09-20](clauderecord/2026-09-20.md) |
| V1.5.0 | 2026-09-18 | 优化：分层归位第二轮 + 重构：拆解聚合头 | [2026-09-18](clauderecord/2026-09-18.md) |
| V1.6.0 | 2026-09-18 | 删除：移除整个循迹功能 | [2026-09-18](clauderecord/2026-09-18.md) |
| V1.7.0 | 2026-09-18 | 新增：`locator_ops9` 定位设备实例 | [2026-09-18](clauderecord/2026-09-18.md) |
| V1.8.0 | 2026-09-18 | 新增：`imu_hwt906` 姿态设备实例 | [2026-09-18](clauderecord/2026-09-18.md) |
| V1.8.1 | 2026-09-18 | 优化：FC_TASK 改经 `imu_hwt906` 实例访问 HWT906 | [2026-09-18](clauderecord/2026-09-18.md) |
| V1.3.0 | 2026-09-17 | 新增 + 优化：分层归位（代码逻辑一行未改） | [2026-09-17](clauderecord/2026-09-17.md) |
| V1.4.0 | 2026-09-17 | 新增：接通任务调度层 | [2026-09-17](clauderecord/2026-09-17.md) |
| V1.0.0 | 2026-09-16 | 新增：初始化四层架构规范 | [2026-09-16](clauderecord/2026-09-16.md) |
| V1.1.0 | 2026-09-16 | 新增：抽象设备层骨架 + `CLAUDE.md` | [2026-09-16](clauderecord/2026-09-16.md) |
| V1.2.0 | 2026-09-16 | 新增：建立 `locator_wheel` 定位设备实例 | [2026-09-16](clauderecord/2026-09-16.md) |
| V1.2.1 | 2026-09-16 | 优化：文档与注释口径修正，无功能改动 | [2026-09-16](clauderecord/2026-09-16.md) |

> 排序规则：本表按**日期降序**（最新在上）、同日按**版本号升序**。
> `clauderecord/` 内的文件按**日期升序**、同日按版本号升序，即版本演化的自然顺序。
