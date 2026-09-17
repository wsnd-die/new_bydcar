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
app/        应用层 + 部分业务层（banyuntask.c、NavigationMecanum.c、BollLocator.c、Mecanum_Move.c、GrayTrace.c）
algorithm/  业务层（Nav_position.c 轮式里程计、mecanum.c、pid.c、angle_ctrl.c、Trace_base.c、Circle_base.c）
hardware/   硬件驱动层（hwt_imu.c、k230.c、emm_v5.c、Send_motor.c、oled.c、sw_uart.c、uart2_tbop10.c …）
device/     抽象设备层 —— 接口 + locator_wheel 实现（OPS9/光流尚未接入）
Core/       CubeMX 生成（main.c、app_freertos.c、外设初始化）
uart/       msp_uart2.c
obsolete/   已停用的驱动（imu660/、hwt101_legacy/、wit_protocol/），不在任何 CMake glob 内，不参与编译
```

> V1.3.0 已做一轮分层归位：`pid` / `angle_ctrl` / `Trace_base` / `Circle_base` 由 `hardware/` 迁入
> `algorithm/`，`wit_protocol` 移入 `obsolete/`。`hardware/` 仍有 30 个文件，其中
> `ColorIdentif.c`（比赛槽位表）、`trace_tune.c`（在线调参）按本规范并不属硬件驱动层，
> 以及 `Common_used.h` 这一聚合头，**均尚未处理**。

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

`CMakeLists.txt:71` 用 `file(GLOB ... CONFIGURE_DEPENDS)` 收集源码，加入的目录为：
`Core/Src`、`app`、`algorithm`、`hardware`、`device`。

**新建 src 目录时必须同时改两处，否则新代码不参与编译：**

1. `CMakeLists.txt` 的 `APP_SOURCES` glob 列表；
2. `CMakeLists.txt` 的 `target_include_directories` 列表。

工程必须用 ARM 交叉工具链编译（`cmake --preset Debug`），用宿主 MinGW 编译会在
`CMakeLists.txt:25` 直接 `FATAL_ERROR`。

---

## 变更日志（持续追加）

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