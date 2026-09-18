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
  ├─ actuators/     emm_v5、Send_motor、block_basic
  ├─ display/       oled、oled_data
  ├─ bus/           sw_uart、uart2_tbop10
  └─ Common_used.h  工程公共头（只聚合 libc + HAL + FreeRTOS）
debug/            调试工具目录（当前为空 —— 原在线调参工具 trace_tune 已随循迹功能删除）
config/           参数配置目录（已建，param_config.h 待第 7.3 节落地）
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

### 6.2 每次修改后必须在第 7 节追加一条记录

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
5. 追加修改记录。

### 7.2 新增位姿融合算法

1. 输入多个 `PoseData_t`，输出融合后的 `PoseData_t`；
2. **禁止修改单个驱动的实现**，融合逻辑独立封装；
3. 支持按时间戳对齐、动态权重调整；
4. 追加修改记录。

### 7.3 参数配置

1. 所有可调参数统一放 `config/param_config.h`（**待建**，见第 2 节）；
2. 使用宏定义并注释含义、取值范围；
3. **禁止在业务逻辑代码中硬编码魔法数字**；
4. 修改参数同样需要记录到变更日志。

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

## 变更日志（持续追加）

#### V1.8.1 - 2026-09-18
- **修改类型**：优化（应用层接线：FC_TASK 改经 `imu_hwt906` 实例访问 HWT906）
- **涉及模块**：应用层 / `app/worker_task.c`
- **修改内容**：
  1. FC_TASK 不再直接调 `HWT_IMU_Init` / `HWT_IMU_Poll`、不再读 `g_hwt_imu_yaw` 全局量，改走 `imu_hwt906` 实例：启动时 `init()` 一次，每拍 `update()` + `get_pose()`；
  2. 删除 FC_TASK 手写的 yaw 差分（`prev_yaw` / `has_prev` / `dt_ms` 状态与 `norm_deg180` 工具函数），角速度直接取实例 `wz`（实例内部同为 yaw 差分、wrap±π、实测 dt，方法与此前手写版一致）；
  3. 量纲换算收进 FC_TASK：实例输出 rad / rad·s⁻¹，角度环按 deg 工作（`angle_ctrl.h`），新增 `RAD2DEG` 宏；`Angle_Update` 入参口径不变（deg、deg/s），对外契约 `g_angle_ctrl_enable` / `g_angle_target_yaw` 未动；
  4. include 由 `hwt_imu.h` 改为 `HWT906.h`。
- **影响范围**：FC_TASK 行为等价——同一数据源（同一 Poll）、同一差分法、同一量纲。里程计（`drv_wheel_odom.c`）与 `NavigationMecanum.c` 仍读 `g_hwt_imu_yaw_rad` 等全局量，由实例 `update()` 内部 Poll 继续刷新，不受影响。失联行为与改前一致（Poll 失败沿用上一拍姿态继续控制，未加 `valid` 保护——属行为改进项，留待实车验证后决定）。
- **验证状态**：已验证（编译链接通过，无 error、无新增 warning；目标文件级确认 `worker_task.c.obj` 内 `FC_Task`（T）已定义且引用 `imu_hwt906`（U）；**但 `.elf` 内仍无 `imu_hwt906` / `FC_Task`——根因见备注 2，与本次接线无关**；未上车验证）。
- **备注**：
  1. **兼容升级**（对外契约未动）。
  2. **【发现既有缺口】`app_freertos.c` 的 `RTOS_THREADS` 区块（112-115 行）只有注释，从未 `osThreadNew` 创建 FC_Task / NLF_Task**——全工程只有 `defaultTask` 一个任务，`worker_task.c` 整段被 `--gc-sections` 回收，V1.8.0 / V1.8.1 的接线当前**不参与运行**。这正是 V1.6.0 日志所述「osThreadNew 未创建 Worker 任务」的现状；V1.4.0 / V1.4.4 声称的创建未体现在当前工作区（重建文件非字节一致所致）。**经用户确认：暂不补创建代码。**
  3. `HWT_IMU_Init` / `HWT_IMU_Poll` 全工程现仅经 `imu_hwt906` 实例调用（FC_TASK 是唯一使用点），V1.8.0 备注 2/3 的二选一问题自然消解。
  4. 待补任务创建后（约 6 行：两个属性 + 两个 `osThreadNew`），`FC_Task → imu_hwt906 → HWT_IMU_Poll` 整链才会进 `.elf`。heap 16384 满足两任务栈需求（1KB + 4KB），仍建议开启 `configCHECK_FOR_STACK_OVERFLOW`（V1.4.0 建议，未做）。

#### V1.8.0 - 2026-09-18
- **修改类型**：新增（`imu_hwt906` 姿态设备实例）
- **涉及模块**：抽象设备层 / 重建 `device/HWT906.c`、`device/HWT906.h`（此前只有 44/159 字节空壳占位，用户已删除，本次重写为真实实现）
- **修改内容**：包装 `hardware/sensors/hwt_imu.c` 驱动，实现 `LocatorDev_t` 四函数：
  1. `hwt906_loc_init()`：转调 `HWT_IMU_Init()`（探测在线 + 当前航向设零点，驱动既有语义），复位软件状态；
  2. `hwt906_loc_update()`：转调 `HWT_IMU_Poll()`（阻塞 I2C3，约 100µs），填 `yaw`（`g_hwt_imu_yaw_rad`，已扣零点）/ `pitch` / `roll`（deg→rad）+ `wz`（rad/s，yaw 差分得到，同 FC_TASK 角度环法；差分前 wrap 到 ±π，避免 yaw 在 ±π 跳变产生 2π 尖峰）+ `valid` + `timestamp`；读失败冻结位姿置 `valid=0`；
  3. `hwt906_loc_get_pose()` 纯读取；`hwt906_loc_is_healthy()` 返回 `valid`；
  4. 文件末尾注册 `const LocatorDev_t imu_hwt906`。
- **影响范围**：仅新增，驱动与业务层零改动。实例名**有意不用 `locator_*` 前缀**：本设备是姿态源不是定位源，`x/y/vx/vy` 恒 0，命名为 `locator_*` 可能被误选为 `active_locator` 造成定位静默失效。
- **验证状态**：已验证。
  1. `cmake --preset Debug` + `cmake --build --preset Debug` 通过，无 error、无新增 warning；
  2. `nm` 目标文件级确认：`imu_hwt906`（R）+ 四个 `hwt906_loc_*`（t）均已定义；
  3. `.elf` 内无 `imu_hwt906`——未被引用被 `--gc-sections` 回收，属预期行为（同 V1.7.0）；
  4. **尚未上车验证运行时行为。**
- **备注**：
  1. **兼容升级**：新增实例，未修改任何已有接口。
  2. **重复 Poll**：`app/worker_task.c:92` 的 FC_TASK 已每周期调 `HWT_IMU_Poll()`；本实例 `update()` 按 `LocatorDev_t` 契约自带 Poll。当前无人调用实例，无实际影响；将来 FC_TASK 若改读本实例，应删掉其直接 Poll，避免同周期双重 I2C 读取。
  3. **重复 Init 语义**：`HWT_IMU_Init()` 会重置航向零点，FC_TASK 启动时已调用（`worker_task.c:85`）；本实例 `init()` 接入时两者应二选一（均在车静止时调用无实质影响，但语义应保持单一）。
  4. 至此三个 `LocatorDev_t` 实例齐备：`locator_wheel`（定位）/ `locator_ops9`（定位，通信接线受 USART3 冲突阻塞，见 V1.7.0 备注 2）/ `imu_hwt906`（姿态）。`active_locator` 仍未建立。

#### V1.7.0 - 2026-09-18
- **修改类型**：新增（`locator_ops9` 定位设备实例，按用户选定方案 A：实例直接建在既有驱动文件内，不新建 `drv_ops9.c`）
- **涉及模块**：抽象设备层 / `device/ops9_g491_uart3.c`、`device/ops9_g491_uart3.h`
- **修改内容**：
  1. 头文件删除悬空 `#include "ops9.h"`（该文件全仓库不存在，此前本文件无法编译；本次一并收进变更记录），改为 `#include "locator_dev.h"`，新增 `OPS9_FRAME_TIMEOUT_MS`（500ms）宏与 `extern const LocatorDev_t locator_ops9;` 声明；
  2. 在既有 OPS9 通信驱动文件**末尾追加** `LocatorDev_t` 适配层与实例，**通信/解码逻辑（ops9_t 状态机、USART3 HAL 粘合）一行未动**：
     - `ops9_loc_init()`：挂接 CubeMX 生成的 `huart3`（`OPS9_G491_UART3_Attach`）并复位软件状态；USART3 外设初始化由 `main.c` 的 `MX_USART3_UART_Init` 完成，此处不重复（同 `locator_wheel` 先例）；
     - `ops9_loc_update()`：调 `OPS9_G491_UART3_GetLatest()` 取最新帧 → 单位换算（mm→m、度→rad，yaw 归一化到 [-π, π]）→ 填 `PoseData_t` 的 `valid` + `timestamp`（`HAL_GetTick()`）；无新帧时按 `OPS9_FRAME_TIMEOUT_MS` 判帧流超时，置 `valid=0` 并冻结位姿，绝不外推；`vx/vy` 恒 0（OPS9 不输出线速度，同 `locator_wheel` 先例）；
     - `ops9_loc_get_pose()` 纯读取、`ops9_loc_is_healthy()` 返回 `s_pose.valid`——四函数签名与 `LocatorDev_t` 契约逐字一致；
     - 文件末尾定义 `const LocatorDev_t locator_ops9`（注册方式同 `drv_wheel_odom.c:157` 的 `locator_wheel`）。
- **影响范围**：仅新增，业务层接口零改动。上层此后可经 `locator_ops9` 以与 `locator_wheel` 相同的四个接口取位姿（m / rad），切换定位源只改 `active_locator` 指针一行。本驱动不再对外裸露 mm/度 的私有格式（`ops9_data_t` 仅内部使用）。
- **验证状态**：已验证。
  1. `cmake --preset Debug` + `cmake --build --preset Debug` 通过（CLion 自带 cmake/ninja），140/140 目标，无 error；`ops9_g491_uart3.c` 仅一条**既有**告警 `s_owned_huart3` 未使用（`-Wunused-variable`，非本次引入）；
  2. `nm` 目标文件级确认：`locator_ops9`（R）+ `ops9_loc_init/update/get_pose/is_healthy`（t）均已定义；
  3. `.elf` 内**无** `locator_ops9`——实例尚未被任何代码引用，被 `--gc-sections` 回收，**属预期行为**（同 V1.4.0 先例，接入 `active_locator` 后自然进入 .elf）；
  4. `.elf` 现为 `text 45068 / data 492 / bss 23180`。**尚未上车验证运行时行为。**
- **备注**：
  1. **兼容升级**：新增实例，未修改任何已有接口。
  2. **【阻塞项】USART3 与外设冲突未解决，通信尚未接线**：`hardware/sensors/k230.c` 同样使用 USART3（`huart3`），且全工程唯一的 `HAL_UART_RxCpltCallback`（`hardware/sensors/QRcode.c:112`）把 USART3 分支派给 `K230_RxProcessByte()`。OPS9 驱动的 `OPS9_G491_UART3_RxCpltCallback()` 目前**零调用**——不接线就没有数据流；若直接接线，两个驱动对同一 `huart3` 各调 `HAL_UART_Receive_IT` 会互斥（`HAL_BUSY`）。需用户决定：OPS9 换 UART / K230 挪走 / 其他方案，之后才能上车验证。
  3. 坐标映射按直通（x/y 即 OPS9 坐标、yaw 即 OPS9 heading）；OPS9 heading 正方向与安装方位未经实测，若与车体系约定（X=前、Y=左、yaw CCW 正，第 3 节）不符，只需调整 `ops9_loc_update()` 内的符号与轴对应，上层无感知。
  4. `active_locator` 尚未建立（第 2 节现状表），实例暂未接入任何任务；接入按 7.1 节第 4 步改指针即可。
  5. 遗留小问题本次未动（见上轮评审）：`s_owned_huart3` 死变量（头文件注释所述 `OPS9_G491_UART3_InitStandalone` 模式未实现）；`OPS9_G491_UART3_SetPose()` 内 `HAL_Delay(10)` 建议换 `osDelay`；`OPS9_G491_UART3_ErrorCallback()` 中断上下文内 `HAL_UART_AbortReceive` 含超时等待风险。

#### V1.6.0 - 2026-09-18
- **修改类型**：删除（移除整个循迹功能）
- **涉及模块**：业务层 / `algorithm/`；应用层 / `app/`；调试 / `debug/`；硬件驱动层 / `hardware/sensors/QRcode.c`
- **修改内容**：按用户决定，**整体移除循迹功能**，共删除 6 个文件：
  1. **`app/GrayTrace.c/h`**（灰度循迹，8 路灰度传感器）。它本就是**死代码** —— `GrayTrace_Update` / `GrayTrace_Update_two` 全仓库零调用者，`$egain/$ffgain` 调参通道收得到字节但永远不被消费。实车跑的是 K230 视觉循迹，不是这条。
  2. **`algorithm/Trace_base.c/h`**（循迹底盘控制，PID / Stanley 两套控制器）。它是活的（经 `NLF_RunFlow` → `Trace_LineFollow`），随功能整体移除。
  3. **`debug/trace_tune.c/h`**（在线调参工具，375 行）。**这是删除前两个的连带结果，非独立决定**：它调的全部参数（`g_tune_angle_*` → `Trace_base` 的 `g_pid_angle`、`g_tune_pos_*` → `g_pid_pos`、`g_tune_gray_*` → `GrayTrace` 的 `g_pid_gray`）都属于被删的两个模块，且它的两个调用者 `Trace_Tune_Service` / `Trace_Tune_Record` 只在 `Trace_base.c` 里被调。两个模块一删，它成为**零消费者、零调参对象**的空壳，且因 `extern pid_type_def g_pid_*` 悬空而无法链接。`debug/` 目录保留待将来放别的调试工具。
- **连带清理**：
  - `hardware/sensors/QRcode.c`：删除对 `Trace_Tune_OnByte` / `GrayTrace_Tune_OnByte` 的 include 与两级中断分流。**驱动层的中断回调不再依赖调试/业务模块**（V1.5.0 备注 4 遗留项之一，就此解决）。
  - `app/worker_task.c`：删除 `Trace_base.h` include 与 `NLF_RunFlow()` 里 `Event_LinFolL` / `Event_LinFolR` → `Trace_LineFollow()` 的分支。
  - `banyuntask.h` 的 `Event_LinFolL` / `Event_LinFolR` **枚举值保留未动** —— 它们是事件词汇表，将来换别的循迹方案时在 `NLF_RunFlow()` 接一个新分支即可。
- **影响范围**：循迹功能整体消失，仅保留 K230 找圆（`Circle_Follow`）与导航（`Nav_RunWaypoints`）。`.elf` 由 `text 44792 / data 476 / bss 23556` 变为 `text 44532 / data 476 / bss 23492`（-260 B）。**注意减幅很小，因为当前 `osThreadNew` 未创建 Worker 任务，应用层整体仍被 `--gc-sections` 回收** —— 不能据此判断删除的影响面。
- **验证状态**：已验证。
  1. `cmake --build --preset Debug --clean-first` 全量重编，无 error；**无新增 warning**（仅剩 `ColorIdentif.c` 两处、`oled_data.c` 三处既有告警）；
  2. `nm` 确认 `Trace_LineFollow` / `Trace_Tune_OnByte` / `GrayTrace_Update` / `g_pid_angle` / `g_pid_gray` / `g_tune_angle_kp` 均已不在符号表中；`nm -u` 为空；
  3. 全仓库 grep 确认除注释里的说明性文字外，无残留引用。
- **备注**：
  1. **不兼容升级**（功能层面）：依赖循迹的调用方需要改用其他方案。对**已保留模块的接口**无影响。
  2. **`hardware/sensors/grayscale.c` 与 `k230.c` 的部分导出函数失去调用者**：`Grayscale_Serial_Read` / `Grayscale_Update` / `K230_GetLineAngle` / `K230_GetPosition` 此前只被 `GrayTrace.c` / `Trace_base.c` 调用。它们是非 static 的驱动 API，不会产生 warning，**本次保留未删** —— 若确认不再需要，可另行清理。
  3. V1.5.0 遗留项 ②（`trace_tune` ↔ `Trace_base` 的双向裸全局耦合）与 ③（`QRcode.c` 中断回调硬编码调参分流）**随本次删除一并消失**，问题不再存在。
  4. 仍需处理：`app/` 内业务模块（`BollLocator` / `Mecanum_Move` / `NavigationMecanum`）向 `algorithm/` 的下沉；`config/param_config.h` 的参数收拢。

#### V1.5.0 - 2026-09-18
- **修改类型**：优化（分层归位第二轮）+ 重构（拆解聚合头）
- **涉及模块**：构建 / `CMakeLists.txt`；硬件驱动层 / `hardware/` 全部；应用层 / `app/`；调试 / 新增 `debug/`；文档 / `CLAUDE.md`
- **修改内容**：代码逻辑一行未改，全部是**位置调整 + include 显式化**。
  1. **`hardware/` 按器件类型分子目录**（此前 30 个文件平铺）：
     `sensors/`（hwt_imu、grayscale、color、collect_ir、k230、QRcode）、
     `actuators/`（emm_v5、Send_motor、block_basic）、
     `display/`（oled、oled_data）、
     `bus/`（sw_uart、uart2_tbop10）。
  2. **新增 `debug/` 目录**，`trace_tune.c/h` 由 `hardware/` 迁入。它是在线调参工具，不属四层中的任何一层。
  3. **`ColorIdentif.c/h` 迁入 `app/`**。它内容是 QR 序号 → 槽位映射表与旋转进度推进，属**比赛策略**而非硬件驱动。
  4. **拆解 `hardware/Common_used.h` 聚合头**（本轮最重要的一项）：
     - 该文件此前把全部 hardware 头、`../algorithm/mecanum.h`、以及全部 app 层头（banyuntask / Mecanum_Move / NavigationMecanum / Nav_position / GrayTrace）一次性 include 进来，使 include 图退化成**完全图** —— 任何一层的任何文件都能看见其它层的任何符号，第 1 节的「驱动与业务完全解耦」形同虚设；
     - 现瘦身为**只聚合底座**：libc + HAL/CMSIS + FreeRTOS/CMSIS-RTOS2 + CubeMX 外设句柄（`gpio/dma/fdcan/i2c/spi/tim/usart.h`，它们是 HAL 层设施，不属四层中的任何一层，保留可免去每个文件重复写一长串）；
     - **13 个 .c 各自补上真正依赖的模块头**（如 `Trace_base.c` 补 `Trace_base.h` —— 它此前连自己的头都没 include，靠聚合头顺带拉进来）。
     - 删除 **12 个全工程无定义的悬空 extern**（`FlagOFMotor` / `FlagOFYuyin` / `Data_uart1` / `Data_uart3` / `buffer` / `buffer_flag` / `buf` / `data_angle` 及 `Uart3_deel` / `Uart1_DMA_IDLE_Start` / `shell_print` / `shell_print3` / `Send_commendyu` / `UART3_Send` / `Guan_dao` —— 整套 UART3/语音子系统已不存在，只剩声明）；
     - 删除零引用的 `arm_math.h`、`use_xing_che`、`ni_he_mode`、`RX_BUF_SIZE`、`DEG_TO_RAD`、`RAD_TO_DEG`。
     - `g_angle_ctrl_enable` / `g_angle_target_yaw` 的 extern 移入 `app/worker_task.h`（其定义处所在模块）。
  5. **3 个"间接 include 聚合头"的头文件一并修正**：`app/BollLocator.h`、`algorithm/Nav_position.h`、`app/ColorIdentif.h` 改为各自只 include 真正需要的头。
  6. **`CMakeLists.txt` 改为 `GLOB_RECURSE`**，并纳入 `debug/`、`config/`（`config/` 目录已建，`param_config.h` 待第 7.3 节落地）。今后在 `hardware/` 下再分目录无需改 CMake；新增**顶层**层目录仍需同时改 glob 与 include 路径两处。
- **影响范围**：**无功能影响**，接口签名一律未改。`hardware/` 由 30 个文件减为 1 个头文件 + 4 个子目录。
- **验证状态**：已验证。
  1. `cmake --build --preset Debug --clean-first` 全量重编，无 error；**无新增 warning**（仅剩 `ColorIdentif.c` 两处、`oled_data.c` 三处既有告警）；
  2. `.elf` 为 `text 44792 / data 476 / bss 23556`，与改动前**完全一致**；
  3. **目标文件级比对**：139 个 `.obj` 经 `objcopy --strip-debug` 后逐字节比对，**全部完全一致** —— 证明搬迁与拆解未改变任何代码生成；
  4. **尚未上车验证运行时行为**（本次无行为改动，风险主要在上车后 include 是否齐全，但编译已覆盖）。
- **备注**：
  1. **兼容升级**，对上层无影响。
  2. **本轮暴露的两个隐藏依赖**（均已修复）：
     - `algorithm/mecanum.c` 用的 `PI` 常量**只定义在 CMSIS-DSP 的 `arm_math.h`** 里。为这一个数学常量而把整个 DSP 库拉进公共头是不合理的，已改用字面量 `3.14159265358979f`（与该宏原值一致）。**注意不要改成 `<math.h>` 的 `M_PI`** —— 它是 `double`，会让 `2.0f * MEC_WHEEL_RADIUS * M_PI` 整个表达式提升为双精度，既变慢又改变舍入。
     - 补 include 时若遗漏各模块头，编译器会给出 `-Wimplicit-function-declaration` 并静默按 `int` 处理返回值（如 `BlockBasic_TurntableTo`、`can_SendCmd`），**不报 error 只报 warning**。
  3. **查告警必须用 `--clean-first`**：本次曾用增量构建检查"无新增 warning"，因相关文件未重编而漏掉了两处隐式声明。增量构建的告警结论不可信。
  4. **`debug/trace_tune.c` 与 `algorithm/Trace_base.c` 的双向裸全局耦合本轮未解**：`trace_tune.c:37-39` 用 `extern pid_type_def` 直接读写 `Trace_base.c` 的 `g_pid_angle` / `g_pid_pos`，`Trace_base.c` 反读 11 处 `g_tune_*` 三元表达式。只搬文件不拆这层耦合，等于把隐式依赖换个目录继续存在。同理 `hardware/sensors/QRcode.c:114,119` 在 USART1 中断回调里硬编码了 `Trace_Tune_OnByte` / `GrayTrace_Tune_OnByte` 两个调参分流 —— 驱动层依赖调试工具。**这两处留待后续单独处理。**
  5. 本轮**未做**：`app/` 内业务模块（`BollLocator` / `GrayTrace` / `Mecanum_Move` / `NavigationMecanum`）向 `algorithm/` 的归位。探查结论：`Mecanum_Move.c` 是「算法 + 驱动」混杂（10 处 `Emm_V5_*` 直调），需按函数切；`BollLocator.c` 算法纯度最高（结构体传参、无全局表），只需把 `TB_position` / `imu_yaw` 裸读换成 `PoseData_t` 入参即可整体下沉；`NavigationMecanum.c` 是「通用规划 + 比赛编排 + 跨任务握手」三合一，需要先定义位姿输入 / 底盘输出 / 阻塞旋转 / 计时四个注入式接口才能切开。

#### V1.4.0 - 2026-09-17
- **修改类型**：新增（接通任务调度层）
- **涉及模块**：应用层 / `Core/Src/app_freertos.c`、新增 `app/worker_task.c`、`app/worker_task.h`；配置 / `Core/Inc/FreeRTOSConfig.h`
- **修改内容**：本工程此前**只有 `defaultTask` 一个空循环任务**（`osDelay(1)`）。`osThreadNew` 在全部 5 个提交里都只出现一次；`task_init()` / `task_send()` / `task_recive()` / `systemEventQueue` **全仓库零调用**；`FC_TASK` / `NLF_TASK` 只存在于注释中（`app/banyuntask.h:6,22`、`hardware/Common_used.h:138`），从初始提交起就未实现。本次按 `app/banyuntask.h` 文件头写明的架构（**驱动源 → defaultTask 调度器 → Worker 任务**）补齐：
  1. **`defaultTask` 改为事件调度器**：`task_recive()` 阻塞收事件 → `NLF_Request(Mode)` 转交 Worker；自身不做任何阻塞式工作。保留原有 PE0 拉低上电动作；顺带移除未使用的 `tx_buf`，消除该文件一处 `-Wunused-variable`。
  2. **新增 `app/worker_task.c/h`**，实现两个 Worker：
     - **`FC_TASK`**（10ms 周期，`osPriorityAboveNormal`）：按 `app/NavigationMecanum.c:184-195` 的既有契约实现角度环。`g_angle_ctrl_enable` **上升沿**调 `Angle_SetTarget()` 复位 PID，之后每拍 `Angle_UpdateTarget()` 连续追踪，`Angle_Update()` 输出的 `cmd_w` 经 `Mecanum_Calc(0, cmd_w)` 下发；**下降沿主动下发零速**——调用方的 `osDelay(20)`「等 FC_TASK 停止输出」等的就是这个。同时承担 HWT906 的周期刷新（全工程周期最短的任务，由它统一 `HWT_IMU_Poll()`）。
     - **`NLF_TASK`**（`osPriorityNormal`，4KB 栈）：线程标志唤醒，执行流程。
  3. **补齐两个悬空全局量**：`g_angle_ctrl_enable` / `g_angle_target_yaw` 在 `app/worker_task.c` 定义。此前它们被 `NavigationMecanum.c` 引用却全工程无定义，因整个 `app/` 被 `--gc-sections` 回收才未暴露成链接错误。
  4. **`configTOTAL_HEAP_SIZE` 3072 → 16384**：原值只够 `defaultTask` 一个线程（栈 512B）+ 空闲任务，加入 Worker 后 `osThreadNew` 会返回 NULL **且不报任何错**，表现为任务静默不跑。
- **影响范围**：**这是本工程第一次让应用代码真正参与运行**。此前 `app/`、`algorithm/`、`hardware/` 的绝大多数函数被链接器回收，`.elf` 只含 HAL + FreeRTOS + libc + `main` + ISR 可达路径。本次 FLASH `44532 → 64084`（+19.5KB），RAM `10704 → 25104`（+14.4KB）。**业务层函数签名一律未改，兼容升级。**
- **验证状态**：
  1. 编译链接通过，无 error、**无新增 warning**（仅剩 `ColorIdentif.c` / `oled_data.c` 的既有告警）；
  2. `nm` 确认 `FC_Task` / `NLF_Task` / `Mecanum_Calc` / `PID_calc` / `HWT_IMU_Poll` / `Send_commandmotor` / `Angle_Update` / `Trace_LineFollow` / `Circle_Follow` / `Nav_RunWaypoints` / `Emm_V5_Vel_Control` 等此前**全部不在符号表内**的函数，本次均已进入 `.elf`；`nm -u` 为空，无未解析符号；
  3. `ucHeap` 实测 `0x4000` = 16384 B，堆大小改动生效；
  4. **尚未上车验证运行时行为。** 堆实际是否够用、`FC_TASK` 周期是否被 `Send_commandmotor()` 内的 `osDelay(5)` 拖长、角度环整定在实测 dt 下是否仍然合适，均需硬件确认。
- **备注**：
  1. **流程入口未接**：`NLF_RunFlow()` 只写死了 Mode → 执行体的映射（`Event_LinFolL/LinFolR`→`Trace_LineFollow`、`Event_FindCircle`→`Circle_Follow`、`Event_Navigation/GoHome`→`Nav_RunWaypoints`、`Event_STOP`→关角度环）。**整条流程的顺序编排、以及第一个事件由谁触发，尚未确定**；`Event_QRCode` / `Event_PickUp` / `Event_PlaceDown` / `Event_STEERING_ROTATE` 四个分支留空待接（各自的执行体 `SetQR()`、`BlockBasic_LiftTo()`、`BL_Update()`、`BlockBasic_TurntableTo()` 都已存在，缺的是入参来源）。
  2. **驱动源仍未接队列**：`task_send()` 目前仍是**零调用**。ISR 侧（`hardware/QRcode.c` 的 `HAL_UART_RxCpltCallback` 等）只置裸标志（`QR_Flag`），需由一个轮询任务翻译成事件。注意 `task_send()` 用的是 `xQueueSend` 而非 `xQueueSendFromISR`，**不能在中断里直接调用**。
  3. **角速度靠差分**：HWT906 只输出欧拉角、无原始角速度（见 `hardware/hwt_imu.h:10-13`），`AngleLoop_Update()` 需要的 `cur_w` 只能由 yaw 差分得到。`FC_TASK` 内用**实测 dt** 而非 `FC_TASK_PERIOD_MS` 常量补偿，因为 `Send_commandmotor()` 内含 `osDelay(5)`，实际周期会大于 10ms。
  4. **建议后续开启 `configCHECK_FOR_STACK_OVERFLOW` 并实现 `vApplicationStackOverflowHook`**：本工程刚接入任务，栈溢出目前是完全静默的，与 `osThreadNew` 返回 NULL 一样难以察觉。
  5. V1.3.0 遗留未做项（`Common_used.h` 拆解、`trace_tune` 迁出至 `debug/`、`app/` 内业务模块归位）本次**仍未动**。

#### V1.3.0 - 2026-09-17
- **修改类型**：新增 + 优化（分层归位，代码逻辑一行未改）
- **涉及模块**：业务层 / `algorithm/`；硬件驱动层 / `hardware/`；构建配置 / **未改动**
- **修改内容**：按第 1 节四层架构对现有源码做**纯位置调整**，判定依据是「是否直接操作 HAL / 外设句柄 / 寄存器」。
  1. **业务层归位**：`pid.c/h`、`angle_ctrl.c/h`、`Trace_base.c/h`、`Circle_base.c/h` 由 `hardware/` 迁入 `algorithm/`。这四个模块**零 HAL 调用、零外设句柄**（`Trace_base.c` 仅用 `HAL_GetTick()` 计时），是纯控制算法，此前误置于硬件驱动层；
  2. **补建接口文件**：新建 `hardware/Send_motor.h`。此前 `Send_motor.c` 是本工程唯一没有头文件的 `.c`，`Send_commandmotor()` 的原型散落在 `Common_used.h:153` 与 `algorithm/mecanum.c:265` 两处**手写 `extern`** 中，接口没有单一出处。现收归头文件，该两处改为 `#include "Send_motor.h"`；
  3. **死代码出编**：`wit_protocol.c/h`（857 行）移入 `obsolete/wit_protocol/`。全仓库零引用（`WitInit` / `WitSerialDataIn` 无任何调用者），功能与 `hwt_imu.c` 重叠。**文件保留**——`obsolete/` 不在任何 CMake glob 内，故不再参与编译。
- **影响范围**：**无功能影响**。`hardware/` 由 40 个文件减至 30 个。被迁移模块的头文件名未变，且 `hardware/` 与 `algorithm/` 同在 include 路径内，`#include "pid.h"` 一类**一律继续解析，调用方零改动**。
- **验证状态**：已验证。
  1. `cmake --preset Debug` + `cmake --build --preset Debug` 全量重编，无 error，**无新增 warning**（仅剩 `oled_data.c` 的 `-Wmissing-braces`、`ColorIdentif.c` 两处、`app_freertos.c` 一处，均为本次未触碰文件的既有告警）；
  2. `.elf` 为 `text 44048 / data 472 / bss 10232`，与改动前**逐位一致**；
  3. **目标文件级比对**（本次采用的更强证据）：改动前 139 个 `.obj` 与改动后 138 个 `.obj`，`objcopy --strip-debug` 后逐字节比对，**除被移出编译的 `wit_protocol.c` 外全部完全一致**。
- **备注**：
  1. **兼容升级**，对上层无影响。
  2. 本次**未改 CMake**：`app/`、`algorithm/`、`hardware/`、`device/` 四目录本就在 `APP_SOURCES` glob 与 `target_include_directories` 内，**目录之间移动源码无需构建配置变更**；但若要在 `hardware/` 下再建子目录，仍须同步改第 8 节所述两处。
  3. **【重要】本工程原有的「`.elf` 字节一致」验证标准偏弱，后续勿单独依赖它。** 本次实测发现：`-Wl,--gc-sections` 会把未被引用的模块整个回收，而 `Core/Src/app_freertos.c:114` 的 `StartDefaultTask` **当前是空循环**（`osDelay(1)`），未调用任何应用层代码，导致 `app/`、`algorithm/`、`hardware/` 的绝大多数函数**根本不在 `.elf` 符号表内**——`nm` 查不到 `Mecanum_Calc`、`PID_calc`、`HWT_IMU_Init`、`Send_commandmotor`、`Emm_V5_En_Control` 等。也就是说 `.elf` 只包含 HAL + FreeRTOS + libc + `main` + ISR 可达路径，大小相同可能只是因为**双方都没被链接**。故此标准只能作为必要不充分条件，重构验证应改用目标文件级比对。
  4. 上述「应用层代码全部被回收」属**既有状态，非本次改动引入**，本次**未做处理**。它意味着当前固件烧录后不会执行任何机器人逻辑，建议优先排查任务调度缺失问题。
  5. 本次**未做**：`Common_used.h` 拆解、`trace_tune` 迁出至 `debug/`、`ColorIdentif` 归入应用层、`app/` 内业务模块（`BollLocator` / `GrayTrace` / `Mecanum_Move` / `NavigationMecanum`）的归位。

#### V1.0.0 - 2026-09-16
- **修改类型**：新增
- **涉及模块**：全架构
- **修改内容**：初始化四层架构规范，定义位姿统一结构体与定位设备抽象接口，制定修改规则与日志规范
- **影响范围**：底盘工程整体框架
- **验证状态**：已验证
- **备注**：初始版本，支持轮式里程计、OPS9、光流三种定位源切换

#### V1.2.1 - 2026-09-16
- **修改类型**：优化（文档与注释口径修正，无功能改动）
- **涉及模块**：抽象设备层 / `device/drv_wheel_odom.c`、`device/drv_wheel_odom.h`；文档 / `CLAUDE.md`
- **修改内容**：修正对"轮式里程计内读取 IMU 航向"这一实现的定性。
  V1.2.0 将其记录为**技术债**（称其"不符合规范 4.2.3 的精神，待后续拆分"），与实际设计意图相反。现改为明确记录为**有意的契约设计**：
  1. 新增「2.1 定位源契约」一节，写明每个定位设备对上层只承诺输出 x/y/yaw 三个量，并明确标注**不要**把 IMU 读取从 `drv_wheel_odom.c` 拆出去；
  2. 修正 `drv_wheel_odom.c` 文件头注释与 `drv_wheel_odom.h` 实例声明处的 `@warning`/说明文字；
  3. 修正 V1.2.0 变更日志的备注 1。
- **影响范围**：**无功能影响**，仅注释与文档。代码逻辑一行未动。
- **验证状态**：已验证（注释改动，重新编译链接通过，产物与 V1.2.0 一致）
- **备注**：**兼容升级**。记录此条的目的是防止后续会话误判该设计为待还债务而去做反向拆分——这正是 CLAUDE.md 存在的意义。

#### V1.2.0 - 2026-09-16
- **修改类型**：新增
- **涉及模块**：抽象设备层 / `device/drv_wheel_odom.c`、`device/drv_wheel_odom.h`、`device/locator_dev.h`；业务层 / `algorithm/Nav_position.c`、`algorithm/Nav_position.h`
- **修改内容**：迁移阶段 0 —— 建立 `locator_wheel` 定位设备实例。
  1. 新增 `device/drv_wheel_odom.c/h`，实现 `LocatorDev_t` 四函数接口。积分算法自 `algorithm/Nav_position.c` 的 `World_position_get()` 逐行迁入，状态（`first` / `prev` / 清零标志）由函数内 `static` 收进驱动内部，外部无法直接触碰；
  2. `algorithm/Nav_position.c` 降级为**兼容适配层**：保留 `World_position` / `World_position_get()` / `World_Reset()` 三个旧符号，内部转调 `locator_wheel`，**调用方一律未改**；
  3. `device/locator_dev.h` 补充 `extern const LocatorDev_t locator_wheel;` 声明。
- **影响范围**：**行为与迁移前逐位一致**，调用方零改动。新增 `is_healthy()` 语义：`update()` 读编码器失败时置 `valid=0` 并冻结位姿（迁移前无此概念，无调用方依赖）。`PoseData_t` 的 `pitch/roll/vx/vy/wz` 本驱动不填充，恒为 0。
- **验证状态**：已验证。`cmake --preset Debug` + `cmake --build --preset Debug` 通过，无 error、无 warning；`.elf` 代码段 `text 44052 / data 472 / bss 10232`，与迁移前**字节级完全一致**，证明重构未改变代码生成。**尚未上车验证运行时行为。**
- **备注**：
  1. `drv_wheel_odom.c` 在驱动内读 `hardware/hwt_imu.c` 的 `g_hwt_imu_yaw_rad` 取航向，这是**有意设计**（见 2.1 定位源契约），非技术债。后续 `locator_ops9` 接入时直接输出 x/y/yaw 即可，对上层契约一致。
  2. `World_position_get()` 的**消费型语义**（每调用一次推进一帧积分）按原样保留，未修复。这是既有行为，修改它会改变运行时表现，故留待阶段 1 由 `update()` / `get_pose()` 分离来根治。
  3. **兼容升级**，对上层无影响。

#### V1.1.0 - 2026-09-16
- **修改类型**：新增
- **涉及模块**：抽象设备层 / `device/pose_data.h`、`device/locator_dev.h`、`CLAUDE.md`、`CMakeLists.txt`
- **修改内容**：
  1. 建立抽象设备层目录 `device/`，定义 `PoseData_t` 与 `LocatorDev_t` 头文件骨架（**仅声明，未实现、未接线**）；
  2. 新增 `CLAUDE.md`，将本规范固化为每次会话自动加载的项目约束；
  3. `CMakeLists.txt` 的源码 glob 与 include 路径加入 `device/`。
- **影响范围**：**仅新增，现有代码零改动**。无 `.c` 文件，不影响编译产物；`active_locator` 等尚未接入任何业务逻辑。
- **验证状态**：已验证（`cmake --preset Debug` + `cmake --build --preset Debug`，139/139 目标编译并链接通过，产出 `build/Debug/bydcar_g491vet6.elf`；FLASH 44532 B / 8.49%，RAM 10704 B / 10.89%。仅有 `hardware/oled_data.c` 的 `-Wmissing-braces` 既有告警，与本次改动无关）
- **备注**：
  1. 规范原文要求驱动放在 `drivers/`，但该目录名在 Windows 上与厂商目录 `Drivers/`（STM32 HAL）大小写冲突、解析为同一目录，故改用 `device/`（对应规范中"抽象设备层 Device"的层名）。**兼容升级**，对上层无影响。
  2. 规范中的 `PoseData_t` 与仓库现存 `app/NavigationMecanum.h` 的 `World_Dir_t` 并存，尚未统一，迁移需单独授权。