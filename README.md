V1.3.0 - 2026-09-17
修改类型：新增 + 优化（分层归位，代码逻辑一行未改）
涉及模块：业务层 / algorithm/；硬件驱动层 / hardware/；构建配置 / 未改动
修改内容：按第 1 节四层架构对现有源码做纯位置调整，判定依据是「是否直接操作 HAL / 外设句柄 / 寄存器」。
业务层归位：pid.c/h、angle_ctrl.c/h、Trace_base.c/h、Circle_base.c/h 由 hardware/ 迁入 algorithm/。这四个模块零 HAL 调用、零外设句柄（Trace_base.c 仅用 HAL_GetTick() 计时），是纯控制算法，此前误置于硬件驱动层；
补建接口文件：新建 hardware/Send_motor.h。此前 Send_motor.c 是本工程唯一没有头文件的 .c，Send_commandmotor() 的原型散落在 Common_used.h:153 与 algorithm/mecanum.c:265 两处手写 extern 中，接口没有单一出处。现收归头文件，该两处改为 #include "Send_motor.h"；
死代码出编：wit_protocol.c/h（857 行）移入 obsolete/wit_protocol/。全仓库零引用（WitInit / WitSerialDataIn 无任何调用者），功能与 hwt_imu.c 重叠。文件保留——obsolete/ 不在任何 CMake glob 内，故不再参与编译。
影响范围：无功能影响。hardware/ 由 40 个文件减至 30 个。被迁移模块的头文件名未变，且 hardware/ 与 algorithm/ 同在 include 路径内，#include "pid.h" 一类一律继续解析，调用方零改动。
验证状态：已验证。
cmake --preset Debug + cmake --build --preset Debug 全量重编，无 error，无新增 warning（仅剩 oled_data.c 的 -Wmissing-braces、ColorIdentif.c 两处、app_freertos.c 一处，均为本次未触碰文件的既有告警）；
.elf 为 text 44048 / data 472 / bss 10232，与改动前逐位一致；
目标文件级比对（本次采用的更强证据）：改动前 139 个 .obj 与改动后 138 个 .obj，objcopy --strip-debug 后逐字节比对，除被移出编译的 wit_protocol.c 外全部完全一致。
备注：
兼容升级，对上层无影响。
本次未改 CMake：app/、algorithm/、hardware/、device/ 四目录本就在 APP_SOURCES glob 与 target_include_directories 内，目录之间移动源码无需构建配置变更；但若要在 hardware/ 下再建子目录，仍须同步改第 8 节所述两处。
【重要】本工程原有的「.elf 字节一致」验证标准偏弱，后续勿单独依赖它。 本次实测发现：-Wl,--gc-sections 会把未被引用的模块整个回收，而 Core/Src/app_freertos.c:114 的 StartDefaultTask 当前是空循环（osDelay(1)），未调用任何应用层代码，导致 app/、algorithm/、hardware/ 的绝大多数函数根本不在 .elf 符号表内——nm 查不到 Mecanum_Calc、PID_calc、HWT_IMU_Init、Send_commandmotor、Emm_V5_En_Control 等。也就是说 .elf 只包含 HAL + FreeRTOS + libc + main + ISR 可达路径，大小相同可能只是因为双方都没被链接。故此标准只能作为必要不充分条件，重构验证应改用目标文件级比对。
上述「应用层代码全部被回收」属既有状态，非本次改动引入，本次未做处理。它意味着当前固件烧录后不会执行任何机器人逻辑，建议优先排查任务调度缺失问题。
本次未做：Common_used.h 拆解、trace_tune 迁出至 debug/、ColorIdentif 归入应用层、app/ 内业务模块（BollLocator / GrayTrace / Mecanum_Move / NavigationMecanum）的归位。