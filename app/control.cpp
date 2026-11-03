/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

/*
 * 学习顺序（四个文件共用；TODO 编号是建议顺序，不是现有代码已经完成）：
 *   TODO1：arm_trajectory.cpp，先完成 interpolate，再写 start/sample/abort。
 *   TODO2：arm_kinematics.cpp，forward -> inverse -> lineReachable -> derivatives。
 *   TODO3：回到 arm.cpp 接通反馈/标定/使能/手动控制，再写 robotControl 的手动分支。
 *          motor_monitor.cpp 的已有底层函数先阅读，不需要重新实现。
 *   TODO4：control.cpp 串起预备动作和四条边，最后再看 motor_monitor.cpp 调参。
 * 每次只做一个小编号；函数的物理排版顺序不等于学习顺序。
 * 本次只补注释；下面的“第几步”是实现提示，原有占位代码仍需你完成。
 */

/*
 * control.cpp 学习导航：本文件决定“当前做什么”，Arm 决定“这一周期目标是多少”。
 * 已有：初始化、看门狗、上电状态机、复位、LED；主要待写的是 robotControl。
 * TODO3 先写标定和手动；TODO4-1 再写预备动作和正方形，不必一次写完。
 *
 * 实际调用：
 *   controlTask：rc.handle -> controlLoop -> allMotorsHandle，约 1 ms 一次。
 *   controlLoop：看门狗 -> 上电状态机 -> 对应分支 -> arm.handle -> LED。
 *   独立 canTask：canTxMonitor，将 Motor 已算好的电流指令打包发送。
 *   robotControl 只是每周期执行一次的函数，不能再写 while 等待轨迹结束。
 *
 * 查接口：app/arm.h、base/remote/remote.h（DR16）、base/remote/wfly.h（RF206S）。
 * 自定义函数看对应 .h/.cpp；HAL 原生函数看工程自带驱动文件开头和函数注释：
 *   mcu/stm32f407/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
 *   mcu/stm32f407/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c
 *   入口分别搜索 HAL_UART_Init、HAL_IWDG_Refresh；无需凭名字猜参数。
 */
#include "app/control.h"
#include "app/arm.h"

#include "iwdg.h"

// #include "app/imu_monitor.h"
#include "app/motor_monitor.h"
// #include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/remote/remote.h"
#include "base/remote/wfly.h"

// 下面是前置声明：让前面的 controlLoop 能调用后面定义的函数，不会在此执行。
void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);

#ifdef RF206S
extern WFLY rc;
#elif defined(DR16)
extern RC rc;
#endif
// extern IMU board_imu;

// extern 引用 interface/robot.cpp 已创建的 arm；这里不再创建第二个机械臂。
extern Arm arm;
BoardLed led;

// robot_state 管整机流程；Arm::mode() 管机械臂使能；两者不是同一个状态。
// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool robot_init_flag = false;
// 遥控器挡位记录
#ifdef RF206S
WFLY::RCSwitch last_rc_switch;
#endif
#ifdef DR16
RC::RCSwitch last_rc_switch;
#endif

// TODO4-1 状态编号含义：0=无自动动作；1=正前往 A 的预备姿态；
// 2=正在 A->B；3=正在 B->C；4=正在 C->D；5=正在 D->A。
// 全局变量能跨周期记住进度；不要在 robotControl 开头每次都重新清零。
uint8_t arm_step = 0;  // 0 空闲，1 预备，2~5 依次走向 B、C、D、A
volatile bool arm_start_ok = false;  // 最近一次轨迹是否启动成功

/*
 * 【01 controlInit】【已实现，阅读即可】配置遥控串口和初始状态。
 * 输入：无；输出：void，写 UART 配置、初始化 LED、设置 robot_state。
 * 谁调用：interface/robot.cpp 的 controlTask，进入周期循环之前一次。
 * 要调用谁：HAL_UART_Init、Error_Handler、BoardLed::init。
 * 不负责创建 RTOS 任务，也不会把机械臂移动到零点。
 */
void controlInit(void) {
  // 第1步：设置串口波特率、字长、停止位、校验和接收模式。
  // 这些值须匹配接收机协议；保留当前板级配置，先不在这里改参数。
  (RC_UART)->Init.BaudRate = 100000;
  (RC_UART)->Init.WordLength = UART_WORDLENGTH_9B;
#ifdef RF206S
  (RC_UART)->Init.StopBits = UART_STOPBITS_2;
#endif
#ifdef DR16
  (RC_UART)->Init.StopBits = UART_STOPBITS_1;
#endif
  (RC_UART)->Init.Parity = UART_PARITY_EVEN;
  (RC_UART)->Init.Mode = UART_MODE_RX;
  (RC_UART)->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  (RC_UART)->Init.OverSampling = UART_OVERSAMPLING_16;
  // 第2步：让 HAL 根据上面的配置初始化硬件；失败进入工程错误处理。
  if (HAL_UART_Init(RC_UART) != HAL_OK)
  {
      Error_Handler();
  }
  // 第3步：初始化灯，再让整机从 STOP 开始；不等于电调物理断电。
  led.init();
  robot_state = STOP;
}

/*
 * 【02 controlLoop】【TODO3-0：先读一遍】每周期的总调度入口。
 * 输入：无，读取遥控连接和开关；输出：void，推进整机和机械臂状态。
 * 谁调用：controlTask 每周期一次；它的下一句才是 allMotorsHandle。
 * 要调用谁：iwdgHandler、robotPowerStateFSM、robotReset/Startup/Control、
 *           Arm::handle、boardLedHandle。
 */
void controlLoop(void) {
  // 第1步：遥控在线就喂看门狗；离线则停止喂狗并要求 STOP。
  iwdgHandler(rc.connect_.check());
  // 第2步：离线或右开关 DOWN 就请求停机，状态机先于业务分支执行。
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == decltype(rc)::DOWN);

  // 第3步：三个分支只走一个；WORKING 才执行你写的 robotControl。
  if (robot_state == STOP) {
    robotReset();
  } else if (robot_state == STARTUP) {
    robot_init_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    robotControl();  // 机器人控制
  }
  // 第4步：执行机械臂周期处理；即便 STOP 也会调用，所以 Arm 内需检查 Disabled。
  arm.handle();
  // 第5步：更新指示灯，返回外层任务；外层随后统一运行电机 PID。
  boardLedHandle();
}

/*
 * 【03 iwdgHandler】【已实现，阅读即可】处理独立看门狗。
 * 输入：iwdg_refresh_flag，当前调用传的是“遥控是否在线”。
 * 输出：void；false 改整机状态为 STOP，true 则刷新硬件看门狗。
 * 谁调用：controlLoop；要调用谁：HAL_IWDG_Refresh(&hiwdg)。
 * 第1步：判断 flag；第2步：选择停机/不刷新，或正常刷新。
 * 注意：右开关 DOWN 但遥控在线时仍会喂狗，停机由上电状态机处理。
 *       电机离线也不直接决定此处喂狗；电机异常由 Arm/Motor 层处理。
 */
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

/*
 * 【04 robotPowerStateFSM】【已实现，阅读即可】只负责整机状态转移。
 * 输入：stop_flag，true 表示请求停止；另读取 robot_init_flag。
 * 输出：void，更新 robot_state；这里本身不调用电机，也不切断供电。
 * 谁调用：controlLoop；要调用谁：无，使用 if/else 判断状态。
 * 第1步：STOP 且无需停机 -> STARTUP。
 * 第2步：STARTUP 遇停机 -> STOP；初始化成功 -> WORKING。
 * 第3步：WORKING 遇停机 -> STOP；其他情况保留原状态。
 * 返回后 controlLoop 根据新状态调用对应处理；不用写 while 等状态变化。
 */
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (robot_init_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

/*
 * 【05 robotReset】【已实现，阅读即可】撤销上层动作并失能。
 * 输入：无；输出：void，清标志、取消动作并更新开关记录。
 * 谁调用：controlLoop 的 STOP 分支；要调用谁：Arm::disable。
 * 注意：STOP 期间每周期都会来，函数应允许反复执行。
 */
void robotReset(void) {
  // 第1步：清初始化标志与正方形进度，防止恢复后接着旧动作跑。
  robot_init_flag = false;
  arm_step = 0;
  arm_start_ok = false;
  // 第2步：请求软件零输出；Arm::disable 内部还要取消轨迹。
  arm.disable();
  // 第3步：记录当前挡位；后续用新旧挡位比较识别一次切换事件。
  last_rc_switch = rc.switch_;
}

/*
 * 【06 robotStartup】【已实现，阅读即可】报告整机启动处理是否完成。
 * 输入：无；返回：当前实现恒为 true；谁调用：controlLoop 的 STARTUP 分支。
 * 要调用谁：当前无。它没有执行机械臂标定、回零或使能动作。
 * 第1步：准备 true；第2步：返回，controlLoop 保存到 robot_init_flag。
 * 随后的状态机周期会据此进入 WORKING，不能据此认为已知机械零位。
 */
bool robotStartup(void) {
  bool flag = true;

  return flag;
}

/*
 * 【07 robotControl】【TODO3-1 -> TODO4-1】把遥控操作翻译成 Arm 命令。
 * 输入：无形参；读 rc.switch_、rc.channel_、arm 的状态和全局动作进度。
 * 输出：void；更新 arm_step/arm_start_ok/last_rc_switch，并调用 Arm 接口。
 * 谁调用：controlLoop 的 WORKING 分支，每周期一次。
 * 要调用谁：
 *   基础：arm.disable/calibrate/enable/addJointReference/trajAbort。
 *   自动：ArmKinematics::inverse、arm.trajStartJoint/trajStart、
 *         arm.trajRunning/trajFinished；必要时读 arm.state/mode。
 * 前置条件：Arm 的反馈/使能/输出链已实现；自动动作还需 TODO1、TODO2 完成。
 */
void robotControl(void) {
    const bool switch_changed = rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r;
  if (switch_changed) {
    arm_step = 0;
    arm_start_ok = false;
    arm.trajAbort();
  }
  if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
    if (arm.enable()) {
      const int16_t channel[2] = {rc.channel_.l_col, rc.channel_.r_col};
      const float max_speed = math::deg2rad(1440.0f);
      float speed[2]{};
      for (int i = 0; i < 2; i++) {
        if (channel[i] > 20) {
          speed[i] = (channel[i] - 20) / 640.0f * max_speed;
        } else if (channel[i] < -20) {
          speed[i] = (channel[i] + 20) / 640.0f * max_speed;
        }
      }
      arm.addJointReference(speed[0], speed[1], 0.001f);
    }
  } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
    const float target[4][2] = {
      {0.100f, -0.100f},
      {-0.100f, -0.100f},
      {-0.100f, 0.100f},
      {0.100f, 0.100f}
    };
    const float segment_duration = 1.0f;
    if (switch_changed) {
      CartesianReference a{};
      a.x = target[3][0];
      a.y = target[3][1];
      float q[2]{};
      arm_start_ok = arm.enable() && ArmKinematics::inverse(a, arm.state().q, 1, q) && arm.trajStartJoint(q[0], q[1], 2.0f);
      arm_step = arm_start_ok ? 1 : 0;
    } else if (arm_step != 0 && !arm.trajRunning()) {
      if (!arm.trajFinished()) {
        arm_start_ok = false;
        arm_step = 0;
      } else if (arm_step < 5) {
        arm_start_ok = arm.trajStart(target[arm_step - 1][0], target[arm_step - 1][1], segment_duration);
        if (arm_start_ok) {
          arm_step++;
        } else {
          arm_step = 0;
          arm.trajAbort();
        }
      } else {
        arm_step = 0;
      }
    }
  } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
    arm.disable();
    arm.calibrate(0.0f, 0.0f);
  } else if (arm.mode() == Arm::Mode::Joint) {
    arm.trajAbort();
  }
  last_rc_switch = rc.switch_;
  // TODO3-1a：先处理“切换挡位”，这是事件，不是持续的动作模式。
  // 第1步：switch_changed = 左挡位与上次不同 || 右挡位与上次不同。
  // 第2步：若变化，arm_step=0，并 arm.trajAbort() 取消上一模式的轨迹。
  //        只在此处处理“变化”；不要每周期无条件 abort，否则轨迹永远走不完。
  //
  // TODO3-1b：先写最小的标定分支（左 MID、右 MID）。
  // 第3步：在对应 if/else 分支调用 arm.disable()，再 arm.calibrate(0,0)。
  //        人先把两杆沿 +X 摆直；calibrate 只登记当前角度，不会自动找机械零点。
  //        检查它的返回值判断反馈/标定是否成功，不能只看 LED 就认为成功。
  //
  // TODO3-1c：再写手动分支（左 UP、右 UP），与标定分支互斥。
  // 第4步：先 arm.enable()；失败时本周期不提交手动目标。
  // 第5步：读取 rc.channel_.l_col/r_col，分别作为 J0/J1 的输入。
  // 第6步：两个速度先初始化为零；摇杆在 [-20,20] 死区内就保持为零。
  //        正侧 speed=(channel-20)/640.f*max_speed_rad_s；
  //        负侧 speed=(channel+20)/640.f*max_speed_rad_s。
  //        .f 保证浮点除法；max_speed 单位 rad/s，速度值由你根据实物设定。
  // 第7步：arm.addJointReference(speed[0],speed[1],0.001f)。
  //        这是逐周期积分位置目标；摇杆归中时速度为零，目标角保持。
  //        不在这里清 PID；enable 已经使能后应直接成功，避免反复重置。
  //
  // TODO4-1a：基础通路正常后，再加自动分支（左 MID、右 UP）。
  // 第8步：定义顶点，单位 m：
  //        A=(0.1,0.1)，B=(0.1,-0.1)，C=(-0.1,-0.1)，D=(-0.1,0.1)。
  //        target[4][2] 按 B、C、D、A 排列；每边用正的 segment_duration 秒。
  // 第9步：只在 switch_changed 时尝试启动预备动作：
  //        创建 A 的 CartesianReference、float q[2]；
  //        依次 arm.enable()、inverse(A,arm.state().q,+1,q)、
  //        arm.trajStartJoint(q[0],q[1],2.f)，任一步失败就不往后执行。
  //        arm_start_ok 保存最终是否成功；成功 arm_step=1，失败 arm_step=0。
  //        可用 && 短路，或逐层 if；都要避免逆解失败后使用未赋值的 q。
  //
  // TODO4-1b：后续周期只检查进度，不再次启动预备动作。
  // 第10步：在未切换挡位且 arm_step!=0 && !arm.trajRunning() 时检查：
  //   10.1 !arm.trajFinished()：表示取消/失败，arm_step=0，arm_start_ok=false。
  //   10.2 正常完成且 arm_step<5：用 target[arm_step-1] 启动下一条直线。
  //        调 arm.trajStart(x,y,segment_duration)，它从上一规划终点开始。
  //        启动成功 arm_step++；失败 arm_step=0、arm_start_ok=false，并取消轨迹。
  //   10.3 正常完成且 arm_step==5：四条边都结束，arm_step=0，保持 A 点目标。
  //        不要调用 disable，否则结束后不再保持；不要自动再次置 1，否则循环画。
  //
  // TODO3-1d：整理退出模式与记录，作为函数结尾。
  // 第11步：其余挡位，若 Arm 已使能，就 arm.trajAbort()，保持最后的位置目标。
  //         右 DOWN 的停机由 controlLoop 在外层处理，不靠这里单独判断。
  // 第12步：last_rc_switch = rc.switch_，最后再更新，下一周期才知道是否切换。
  //         若中途 return，要保证开关记录不会漏更新；否则会一直认为刚切换。
  //
  // 状态串起来读：
  // 切入自动挡 -> step1预备 -> step2到B -> step3到C -> step4到D -> step5到A -> step0。
  // trajFinished 表示最终参考已输出，不表示实物误差已经达标；PID 继续追踪最终目标。
  // 每周期 robotControl 先执行，arm.handle 后执行；本周期结束标志通常下周期才被这里看到。
  // 学完自查：一直保持自动挡只能画一次；中途切挡能取消；取消后不会擅自接下一边。
  // 当前仍为空函数；上面的步骤需要你逐段写，不会由注释自动生效。
}

/*
 * 【08 boardLedHandle】【已实现，暂时跳过细节】把整机状态显示在灯上。
 * 输入：读取 robot_state、遥控挡位、板型编译宏；输出：void，更新 LED。
 * 谁调用：controlLoop；要调用谁：BoardLed 的设色、模式、handle 等方法。
 * 第1步：#ifdef 选择编译哪套板型灯接口；它不是运行时的 if。
 * 第2步：STOP 红色、STARTUP 启动提示、WORKING 按挡位区分闪烁次数。
 * 第3步：DBC 分支用 led.handle 推进灯效；其余接口按已有板级实现。
 * 注意：灯反映整机/挡位，不直接证明标定、轨迹启动或电机反馈成功。
 */
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);  // red
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);  // yellow
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);  // blue
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setModeBlink(1);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setModeBlink(2);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setModeBlink(3);
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左上右中
      led.setModeBlink(4);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左中右中
      led.setModeBlink(5);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左下右中
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);  // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);  // red+green
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1);  // green
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2);  // green
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3);  // green
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左上右中
      led.setLED(false, true, 4);  // green
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左中右中
      led.setLED(false, true, 5);  // green
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左下右中
      led.setLED(false, true, 6);  // green
    }
  }
#endif
}
