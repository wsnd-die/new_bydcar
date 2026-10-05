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
algorithm/        业务层：mecanum、pid、angle_ctrl、Circle_base、Nav_position、arc_path
device/           抽象设备层：PoseData_t + LocatorDev_t 接口 ＋ locator_wheel 实现（OPS9/光流尚未接入）
hardware/         硬件驱动层，按器件类型细分：
  ├─ sensors/       hwt_imu、grayscale、color、collect_ir、k230、QRcode、key（V1.18.0，待接线）
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
> V1.12.0 **补上了任务创建**：`FC_TASK`（AboveNormal，10ms，兼 HWT906 的唯一轮询者）
> 与 `NLF_TASK`（Normal，阻塞式流程）已由 `MX_FREERTOS_Init()` 创建，
> defaultTask 也改回 `task_recive() → NLF_Request()` 调度器循环，两个任务不再被
> 回收。**但驱动源仍然空**：`task_send()` 依旧零调用，所以 `NLF_TASK` 永久阻塞在
> `osThreadFlagsWait`、`NLF_RunFlow()` 一次也没执行过 —— 这条链目前只通到调度器，
> 还跑不通一条完整流程。
>
> V1.17.0 **把整条比赛流程编排进了 `NLF_RunFlow()`**（原先只是个 Mode → 执行体的
> 空壳），并顺带修掉三处让流程根本跑不起来的断点（V1.17.1）。编排取自一份
> **不在本仓库 git 历史里**的旧 8 任务版 `app_freertos.c`（`NLFKION` / `ColorFunion` /
> `BsRtFunion` / `Navesafter_mode` 等符号在全历史零命中，是另一份副本），
> 只参考其**流程语义**，不是恢复其代码。要点：
>
> - **不再扫二维码**，奖杯放置顺序与槽位颜色改为 `worker_task.c` 顶部的硬编码默认值
>   （每个表都标了「★ NX 接入点」），等上位机 NX 的回传报文接入后只改
>   `NF_FlowSeed()` 一处；
> - **循迹两段（`Event_LinFolL` / `Event_LinFolR`）是打桩** —— 循迹控制器
>   `algorithm/Trace_base.c` 已随 V1.6.0 删除，本次不恢复；
> - **`Nav_FeDuanPoint()` / `Nav_CalibrateAfterTrace()` 也是打桩** —— 它们此前
>   只有声明没有定义，调用即链接失败（V1.17.2 补的）；
> - 默认 `NF_AUTOSTART 1`：**上电即自动开跑**。当前导航/循迹都是打桩所以车不动，
>   但那两段一旦补上真实现，这个开关就等于"上电发车"。
>
> V1.6.0 **移除了整个循迹功能**（`GrayTrace` / `Trace_base` / `trace_tune` 共 6 个文件），
> 只保留 K230 找圆与导航。V1.5.0 遗留的 ② `trace_tune` 双向耦合、③ `QRcode.c` 中断
> 回调硬编码分流，随之消失。
>
> **仍待处理**（按优先级）：① `app/` 内三个业务模块向 `algorithm/` 的下沉（需先定义
> 位姿输入 / 底盘输出 / 阻塞旋转 / 计时四个注入式接口）；② `config/param_config.h`
> 的参数收拢（V1.17.0 的硬编码默认值届时一并迁入）；③ 确认 `grayscale.c` / `k230.c`
> 里失去调用者的驱动 API 是否还要保留；④ **任务职责梳理** —— `FC_TASK` 的文档说它是
> 10ms 角度环，实际它只发 NX 找圆模式，真正的角度环跑在 `angle_Task`（优先级 8）里，
> 而 `NLF_TASK` 是 40，与文档要求的"角度环必须能抢占流程任务"相反（V1.17.1 备注 1）；
> ⑤ `Nav_FeDuanPoint()` 与 `g_waypoints[]`（**17 个点，已随 V1.19.2 换成现场
> 示教值**）均已落地，但：**第 11 个点仍是旧占位值待示教**；`Nav_FeDuanPoint()`
> 的调用方 `NF_Stage_Navigation()` **忽略其返回值**，加上"失败不推进"的语义，
> 某个点持续失败会让流程永久卡在 Navigation 阶段（V1.19.0 备注 1）；
> 且**流程只提供 11 次中继站访问，走不完 17 个点**（V1.19.2 第 5 条）。
> **`worker_task.c` 的 `NF_STAGES[]` 当前是【临时单站表】，测完必须还原。**
> 仍待补：`Nav_CalibrateAfterTrace()`、循迹两段；
> 另有 `Nav_MoveBody()` 实为世界系增量而非文档所称车体量（V1.19.0 第 5 条）、
> `Nav_GoToWorld()` 借走 `g_angle_ctrl_enable` 后不恢复；
> ⑥ **确认 PB0 限位开关的有效电平** —— `block_basic.c:295` 的 `BPlace_SetZero()`
> 判据是「低 = 未到位」，与 V1.18.0 按键驱动假定的「按下 = 高」相反，
> 而 `gripper_task` 里 `while (!BPlace_SetZero());` 是**死等**，配错会让该任务
> 永久阻塞。另外 `.ioc` 里 PB0 写的是 `GPIO_PULLUP`、生成的 `gpio.c` 却是
> `GPIO_NOPULL`（改过 `.ioc` 没重新生成），PA0 则完全没配 PuPd —— 两个脚目前都是浮空输入。

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
| 姿态来源 | `hardware/hwt_imu.c` | 维特 HWT906（**九轴**），I2C3，航向取 `g_hwt_imu_yaw_rad`；V1.11.0 起一次事务读回 0x34~0x40 整块，加速度/角速度/磁场/温度也一并带回 |

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

    // ---- V1.11.0 追加（前向兼容，只能往后加）----
    float ax;         // X轴线加速度，单位：m/s^2，传感器本体坐标系
    float ay;         // Y轴线加速度，单位：m/s^2
    float az;         // Z轴线加速度，单位：m/s^2
} PoseData_t;
```

> `ax/ay/az` 目前只有 `imu_hwt906` 会填；`locator_wheel` / `locator_ops9`
> 无加速度数据源，恒为 0（同 `vx/vy` 先例）。

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
| V1.26.1 | 2026-10-06 | 修复：`TT_RotateReset` 定义被删导致链接失败 —— 恢复定义（`TT_SeekBlock` 游标外提为文件级 `s_seek_slot`）；顺带修 `slot>4` 越界判据（放行 `slot==4` 读 `T1[4][*]`） | [2026-10-06](clauderecord/2026-10-06.md) |
| V1.26.2 | 2026-10-06 | 优化：清理 `g_tt_default_color`/`g_tt_rotate_idx` 两个未用静态量 + `TogetPos` 一行三 `if` 的 misleading-indentation 告警 | [2026-10-06](clauderecord/2026-10-06.md) |
| V1.24.0 | 2026-10-05 | 新增：**航向误差死区** `NAV_YAW_DEADBAND`(≈1°) —— 压住 Kp=5 把 OPS9 航向噪声放大成来回角速度指令的抖；只喂控制器，到达判据仍用**原始** `eyaw` | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.24.1 | 2026-10-05 | 新增：**平移轴速度规划**（√(2·a·e) 当参考 + 8mm 位移死区）+ 到位速度门限 `NAV_ARRIVE_VMAX`；**修复软启动斜坡被丢弃**（算完 `vx_cmd` 又用未限幅原值）、**恢复被写死为 0 的 Y 轴** | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.24.2 | 2026-10-05 | 修复：**√ 曲线只能当"上限"、不能当"参考"** —— 改成"近场线性 P + 远场制动上限"并删掉位移死区（原版到点来回晃，比纯 P 还差） | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.24.3 | 2026-10-05 | 修改：**回家只走最后一点**（原 `Nav_RunWaypoints()` 把整张表再走一遍 → 现场"连续跑两遍"）；`NF_Start()` 判返回值（表跑完不再发未初始化阶段号）；`Nav_GoToWorld` 加 ARRIVE/TIMEOUT 日志与 `ovs` 过冲量 | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.24.4 | 2026-10-05 | 新增：**采集点到位后向前蹭料** `NF_CreepForward()` —— 车体系低速顶进，停车判据＝进料口"完全进入"(遮光→恢复，自带边沿状态不碰 collect_ir 的 static)；`NF_CREEP_WP[]` 列 8 个点位；新增 `Nav_LastWaypointNo()` | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.24.5 | 2026-10-05 | 修复：**收完第 1 个奖杯后跳掉第 2 个点位** —— 进料打断迟到 50~150ms（采集侧 IR 去抖）打到**下一段**头上而游标照推；`NF_ABORT_ON_FEED` 默认 **0**（分点+蹭料模式下不需要打断），旧语义连同 `g_nav_running` 留在宏后面 | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.24.6 | 2026-10-05 | 新增：**放置解锁** —— `Place()` 加 `slot` 参数并 `GripperRelease(slot)`（原来手写那行少传参数 → **整个工程编译不过**）；新增 `TT_CurrentSlot()`（物料是逐个放的，不能一次全松） | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.25.0 | 2026-10-05 | 新增：**丝杆绝对位置指令** `BlockBasic_LiftToAbs()` / `LiftSync()`（旧相对版保留，两版共用 `lift_current`）；5 处调用点迁到等价值 3/28/43/5，`Place()` 内"下降 5"**保持相对**；顺带修正旧注释里方向写反（实为 `UP = 0`） | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.25.1 | 2026-10-05 | 修改：**启动键改长按** —— 驱动新增 `Key_WasLongPressed()`（稳定按下 ≈1s 报一次，读后清），`ops9imu_fuction` 触发点由 `Key_WasPressed` 换成长按；KEY_LIMIT 语义不变 | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.25.2 | 2026-10-05 | 修改：`g_waypoints[]` 1~16 号点按现场重新示教值整体更新（15/16 摆放点 yaw 由占位 0 变为 ≈47°/49°，7 号 e 点西移 0.168 m）；17 号回家点未动 | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.26.0 | 2026-10-05 | 新增：**定半径圆弧 `Nav_Cricle()`** —— OPS9 位置闭环（圆心由起点推得，切向匀速+径向 P 修正+航向前馈 v/R；`Nav_Cricle_t` 左/右弧，扫够角度才退出，支持打断/超时） | [2026-10-05](clauderecord/2026-10-05.md) |
| V1.20.4 | 2026-10-02 | 修复：**MSP 帧解析器与发送端格式对不上**，颜色恒为兜底值 —— 按真实帧 `A3<color><r><g><b>B3` 重写状态机，颜色直接取帧头后的字符；RGB 段因 `%d%d%d` 无分隔符**原理上切不开**故丢弃；取到色打印 `[MSP] color='r' -> RED` | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.20.5 | 2026-10-02 | 修改：`MSP_Color_Init()` 调用点归位到 `main.c` 的 `USER CODE BEGIN 2`；**顺带修掉一处重复调用**（`usart.c` + `app_freertos.c` 各一份，第二次会 memset 后 `HAL_BUSY` 退出） | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.20.6 | 2026-10-02 | 修改：`MSP_Color_Init()` 按用户要求从 `main.c` 移回 `app_freertos.c` 的 `USER CODE BEGIN Init`（**仍只保留一处调用**，V1.20.5 修掉的重复调用未恢复） | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.20.7 | 2026-10-02 | 新增（**临时调试**）：`MSP_Color_DebugPoll()` —— 每收到一整帧就打印 `[MSP-RX] color='r' rgb_raw="2003040"`，挂在 `gripper_task` 循环里与采集流程**解耦**；数字段原样打印不切分（`%d%d%d` 无分隔符无定宽，切不开） | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.21.0 | 2026-10-02 | 新增：**奖杯名次判断** —— `wait_trophy_rank()` 轮询 `NX_GetTrophyRank()`（消费式，全工程仅此一个调用点，`FC_TASK` 里抢帧的调试打印已摘掉），存进 `g_tt.trophy[]`（补 `TT_SetTrophy`/`TrophyAtSlot`）；⚠ 分支开关 `k` 恒 false，**奖杯分支目前是死代码** | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.21.1 | 2026-10-02 | 修改：摆放阶段「名次→槽位」改**反查 `g_tt.trophy[]`**（新增 `SlotByTrophy()`）—— 旧的 `NF_TROPHY_SLOT[]`（冠军→槽3 固定表）与「第 N 个进槽 N」的收集顺序对不上，会拿错奖杯；该表已**删除** | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.21.2 | 2026-10-02 | 修改：奖杯名次改为**等待期间投票** —— `wait_block_entered()` 加 `rank_out` 出参，等物块进来的同时累计 NX 名次的票、进来时取多数（同票取名次靠前，零票写 0），**夹住之前就定好**；删除 `wait_trophy_rank()` | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.21.3 | 2026-10-02 | 修复：NX 奖杯名次**实际回的是字母 `'a'/'b'/'c'` 不是 `'1'/'2'/'3'`**（按数字判断导致每帧都被丢弃、票恒为 0）；新增 `trophy_char_to_rank()` 映射 a=冠军 b=亚军 c=季军 | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.21.4 | 2026-10-02 | 修改：采集分支开关由静态 `k` 改为**调用方传参** —— 新增 `BlockCollectStage_t` + `BlockCollect_SetStage()`，`NF_Stage_Collect(stage)` 由 `Event_Collect_L`/`_R` 分别传 `COLLECT_MATERIAL`/`COLLECT_TROPHY`；分支**不再自动翻转** | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.22.0 | 2026-10-02 | 修复：`Place()` 的**坐标系错配** —— `fwd`/`left` 是车体量却被当世界系增量交给 `Nav_MoveBody()`（只有车头朝世界 +X 时才对）；改为按 `Self_Dir.yaw` 转成世界系再叠加（与 `Nav_GoToWorld` 的「世界→车体」互逆） | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.22.1 | 2026-10-02 | 修改：`Place()` 的移动**改回位置模式开环定距**（V1.22.0 的闭环方案被取代）—— 新增 `Mecanum_MoveBodyPos()`（按车体量算四轮脉冲，`Emm_V5_Pos_Control` 发完就等，不读编码器）；旧的 `Mecanum_Move.*` API 本仓库已删，本次是按同思路重写的精简版 | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.22.2 | 2026-10-02 | 修改：`Mecanum_MoveBodyPos()` 由**盲等固定时间**改为**轮询 `Emm_V5_Is_Reached()` 到位标志**，`wait_ms` 语义变"到位超时"、返回值变 `bool`；⚠ 下发后需先延时 `MEC_POS_START_DELAY_MS` 再查（到位标志是上一次的残值，立刻查会误判） | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.23.0 | 2026-10-02 | 修改：**收集与导航解耦** —— `NF_Stage_Collect()` 删掉最长 120s 的等待循环、只挂请求就返回；新增跨任务打断标志 `g_route_abort`（IR 进料→打断当前路线段改奔下一点），`Nav_GoToWorld`/`Arc_Run` 均支持；路线源可用 `NF_ROUTE_ARC` 编译期切成圆弧模式。⚠ 解耦后 UART5 总线在 NLF/gripper 两任务间无互斥（当前单站表下安全） | [2026-10-02](clauderecord/2026-10-02.md) | 修复：**MSP 帧解析器与发送端格式对不上**，颜色恒为兜底值 —— 按真实帧 `A3<color><r><g><b>B3` 重写状态机，颜色直接取帧头后的字符；RGB 段因 `%d%d%d` 无分隔符**原理上切不开**故丢弃；取到色打印 `[MSP] color='r' -> RED` | [2026-10-02](clauderecord/2026-10-02.md) |
| V1.19.7 | 2026-10-01 | 优化：精简 `arc_path.c/h` 注释（代码逻辑一行未改）；保留两条 `@warning` | [2026-10-01](clauderecord/2026-10-01.md) |
| V1.19.8 | 2026-10-01 | 修改：圆弧改为**自持指令源 + 纯几何开环** —— `Arc_Run()` 自己 `Mecanum_Calc(v, v/R)` + `Send_commandmotor`，不再走 `g_angle_ctrl_*`（worker_task.c 已删其定义，原先**链接失败**）；⚠ 航向无反馈，轨迹会偏 | [2026-10-01](clauderecord/2026-10-01.md) |
| V1.20.0 | 2026-10-01 | 新增：**物块收集** —— 新建 `hardware/sensors/msp_color.*`（USART2 DMA-IDLE 收 `'r'`/`'b'`）+ `app/block_collect.*`（收集状态机，见 V1.20.1）；`block_basic` 加夹爪动作与形状判定；`TT_t` 末尾追加 `shape/raw_angle/collected`；启动键 PA0 改发 `Event_Collect`。⚠ 阈值/时延全为占位值，**实机未验证** | [2026-10-01](clauderecord/2026-10-01.md) |
| V1.20.1 | 2026-10-01 | 修改：**V1.20.0 的设计修正** —— 采集不另起任务，改为 `gripper_task` 循环里轮询 `BlockCollect_Poll()`；删掉 `blockcol` 任务与那 2KB 栈 | [2026-10-01](clauderecord/2026-10-01.md) |
| V1.20.2 | 2026-10-01 | 修复：`Slot_Shape[]` 定义在头文件里导致 **multiple definition 链接失败** —— 移入 `block_basic.c` 并 `static const`；`ShapeFromRaw()` 补槽号越界保护（改为逐槽阈值） | [2026-10-01](clauderecord/2026-10-01.md) |
| V1.20.3 | 2026-10-01 | 修改：**采集时序重排** —— 夹和识别错开一拍（夹紧→转一步→才读形状/颜色，因传感器在转过一步的位置）；圆锥单走一段不夹不识别；第 5 个转 `BLOCK_TURNTABLE_DOOR_DEG`(340°) 关门 | [2026-10-01](clauderecord/2026-10-01.md) |
| V1.19.6 | 2026-09-30 | 修改：`g_waypoints[]` 9~16 点按现场重新示教值更新（11 号点补真实值，摘除旧占位警告） | [2026-09-30](clauderecord/2026-09-30.md) |
| V1.19.4 | 2026-09-29 | 新增：`Scs0009_ReadRaw()` —— 读 SCS0009 位置（走 `SCS_FeedBack` 缓冲区路径以**绕开字节序竞态**）；**零调用者，待接线** | [2026-09-29](clauderecord/2026-09-29.md) |
| V1.19.5 | 2026-09-29 | 修复：**回读跑一会儿就恒为 -1** —— 根因是厂商 `rFlushSCS()` 是假刷新（只延时不清 RX）导致残留错位级联；`ftBus_Delay()` 补 RX 排空 + 清 ORE/FE/NE，`Scs0009_ReadRaw()` 改直读 2 字节（回帧 21→8 字节） | [2026-09-29](clauderecord/2026-09-29.md) |
| V1.17.0 | 2026-09-28 | 新增：把整条比赛流程编排进 `NLF_RunFlow()`；不再扫二维码，奖杯顺序/槽位颜色改用硬编码默认值（标了 ★ NX 接入点）；循迹两段打桩 | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.17.1 | 2026-09-28 | 修复：调度链三处断点 —— `nlfTaskHandle` 恒 NULL（`NLF_Request` 从没发过标志）、`NLF_TASK` 栈实际 1KB 与声明不符、任务名 `findcircle_TASK`/`nav_task` 误导 | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.17.2 | 2026-09-28 | 新增：`Nav_FeDuanPoint()` / `Nav_CalibrateAfterTrace()` 此前有声明无定义（调用即链接失败），补成打桩 | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.17.3 | 2026-09-28 | 修复：`Nav_MoveBody()` 声明 `bool` 却无 `return`（UB） | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.18.0 | 2026-09-28 | 新增：按键消抖驱动 `hardware/sensors/key.*`（PB0 限位 / PA0 启动）；连续采样确认法；**零调用者，待接线** | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.19.0 | 2026-09-28 | 新增：`Nav_FeDuanPoint()` 从打桩补成真实现（一次一步推进）+ 恢复 `g_waypoints[]` 15 个点（**旧场地坐标，待复核**）；新增 `NAV_DEG2RAD`；更正 `Nav_MoveBody()` 文档（世界系非车体系）、标注 5 个无定义声明 | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.19.1 | 2026-09-28 | 修改：修正 OPS9 坐标系映射（实测差一个 −90° 旋转，`x_w=y_o, y_w=−x_o`；det=+1 故 yaw 不反号）；**平移段已上机验证准确**，转向段与 **yaw 零点**仍未标定 | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.19.2 | 2026-09-28 | 修改：`g_waypoints[]` 换成 17 个**现场示教值**（旧表属里程计坐标系，在 OPS9 系下无效）+ 字面量补 `f` 消 49 处窄化告警；**【临时】`NF_STAGES[]` 换单站表专测分点导航 —— 测完必须还原** | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.19.3 | 2026-09-28 | 修改：取消上电自动开跑（`NF_AUTOSTART` 1→0），改由**启动键 PA0 触发**；触发放在 `KEY_TASK` —— 原先放在 `StartDefaultTask` 的位置因 `task_recive()` 用 `portMAX_DELAY` 而**根本不会执行**。⚠ PA0 仍浮空，可能自发车 | [2026-09-28](clauderecord/2026-09-28.md) |
| V1.16.0 | 2026-09-27 | 修改：转盘舵机由 TIM3_CH2 PWM 改为 UART5 上的 STS3032 总线舵机（`SCS_WritePosEx`）；删除 `block_servo_write()` | [2026-09-27](clauderecord/2026-09-27.md) |
| V1.14.0 | 2026-09-26 | 删除：TBOP 里程计自动标定整块（死代码 + 跨层违规）；`Odometry_Apply_Calib` 剥离为单分支 | [2026-09-26](clauderecord/2026-09-26.md) |
| V1.14.1 | 2026-09-26 | 修改：USART3(OPS9) 接收改 DMA+IDLE；`HAL_UARTEx_RxEventCallback` 收归 `usart.c` 做唯一分发器 | [2026-09-26](clauderecord/2026-09-26.md) |
| V1.14.2 | 2026-09-26 | 修复：I2C3 改 DMA 读后未接通 I2C3_EV/ER 中断（传输永不完成）；补 NVIC + IRQHandler + 信号量完成同步 | [2026-09-26](clauderecord/2026-09-26.md) |
| V1.15.0 | 2026-09-26 | 新增：世界系位置闭环（三轴 PD + 软启动 + OPS9 反馈）；`Mecanum_Vel_Execute` 执行器；接通 `Nav_GoToWorld`/`Nav_RunWaypoints` | [2026-09-26](clauderecord/2026-09-26.md) |
| V1.15.1 | 2026-09-26 | 修复：`mecanum.h` 声明 `Mecanum_Calc_V`→`Mecanum_Calc`（与实现对齐，worker_task 编译错误） | [2026-09-26](clauderecord/2026-09-26.md) |
| V1.15.2 | 2026-09-26 | 修改：UART4 接收改 DMA-IDLE + 补完成回调接线；`uart/NX_uart4.*`→`NX_uart.*`；GBK→UTF-8 编码归一 | [2026-09-26](clauderecord/2026-09-26.md) |
| V1.11.0 | 2026-09-25 | 新增：HWT906 读全 0x34~0x40 九轴（块读 + 退化回退）；`wz` 改取陀螺仪；`PoseData_t` 追加 `ax/ay/az` | [2026-09-25](clauderecord/2026-09-25.md) |
| V1.12.0 | 2026-09-25 | 新增：接通任务流程 —— 创建 FC_TASK / NLF_TASK，defaultTask 改走调度器循环；撤 V1.11.0 临时脚手架；堆 8096→16384 | [2026-09-25](clauderecord/2026-09-25.md) |
| V1.13.0 | 2026-09-25 | 新增：定半径圆弧 + 切线航向 —— `algorithm/arc_path`；契约追加线速度与前馈角速度；`Event_ArcRun` | [2026-09-25](clauderecord/2026-09-25.md) |
| V1.10.7 | 2026-09-23 | 修复：SCS0009 总线字节序错误（`setEnd` 全局量按系列切换）；修正中位常量 | [2026-09-23](clauderecord/2026-09-23.md) |
| V1.10.6 | 2026-09-22 | 优化：精简 `gripper_task` 调试脚手架；建立 STS3032+SCS0009 双系列 6 舵机骨架 | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.5 | 2026-09-22 | 新增：广播动作验证发送通路（调试用，会让舵机动作） | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.4 | 2026-09-22 | 新增：PD2 悬空/被驱动判别探针；修复环回超时判据恒假 | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.3 | 2026-09-22 | 修复：`ftUart_Send` 去掉每帧 TE 翻转，与厂商实现对齐 | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.2 | 2026-09-22 | 新增：`gripper_task` 接舵机总线自检；修复栈溢出检查失效、栈/堆不足 | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.1 | 2026-09-22 | 新增：按 STS3032 补 EPROM 解锁对与回读包装（20→35 个） | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.0 | 2026-09-22 | 新增：飞特 SCS 总线舵机驱动包移植（UART5 / 1M） | [2026-09-22](clauderecord/2026-09-22.md) |
| V1.10.1 | 2026-09-22 | 修复：删除重复的 `HAL_UARTEx_RxEventCallback` 定义（链接失败） | [2026-09-22](clauderecord/2026-09-22.md) |
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
