/**
 ******************************************************************************
 * @file    motor_monitor.cpp/h
 * @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
 * @author  Spoon Guan
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
 * motor_monitor.cpp 学习导航：这里的管理函数已经写好，先读，不要当成填空题。
 * TODO3-0 阅读重点：J0/J1 配置 -> ID 表 -> allMotorsInit -> allMotorsHandle。
 * TODO4-2 等标定、手动、轨迹逻辑连通后，再根据目标/反馈曲线调 PID。
 *
 * 两条通路要分清：
 *   输出：Arm::writeMotorCommands 保存目标 -> allMotorsHandle
 *       -> Motor::handle 算 PID、保存 intensity_ -> canTask 打包发送。
 *   反馈：CAN 中断 -> HAL_CAN_RxFifo0MsgPendingCallback
 *       -> motorsCanRxMsgHandle -> DJI 驱动解包 -> Motor::rxCallback。
 * “保存目标”“计算控制量”“发 CAN”是三个不同动作。
 * Arm 的角度是 rad，Motor 的角度是 deg；换单位在 Arm 输出/读取处完成。
 *
 * 本地查定义，比猜名字更直接：
 *   base/motor/motor.h/.cpp：构造参数、模式、handle、setAngleSpeed、reset。
 *   base/common/pid.h/.cpp：PID 的 6 个参数、calc、reset 的实际算法。
 *   base/motor/driver/dji_motor_driver.cpp：ID 配置、CAN 解包和电流指令打包。
 *   interface/robot.cpp、interface/callback.cpp、app/can_monitor.cpp：谁来调度。
 * 原生 CAN API：工程 HAL 驱动 Src/stm32f4xx_hal_can.c，搜 HAL_CAN_AddTxMessage。
 * 下面停机函数中的 POWEROFF 是软件零控制量，不是切断电调电源。
 */
#include "app/motor_monitor.h"
#include "lib/arm_math/arm_math.h"

// Motor parameter config
// 电机参数配置

// 以下底盘/云台/摩擦轮/拨弹电机整段都被 // 注释，属于示例，不会创建电机。
// 本项目先跳到后面的 J0、J1，避免把模板示例当成当前硬件配置。
// Chassis motor 底盘电机
// const PID chassis_wheel_spid(40, 1, 10, 1000, 16384);
// Motor CMFL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMFR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMBL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMBR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid

// Gimbal motor 云台电机
// Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,   // type, ratio, method
//           PID(25, 0.1, 10, 10, 1800),                // ppid
//           PID(400, 2, 0, 500, 30000),                // spid
//           true);                                     // use kf
// Motor GMP(Motor::GM6020, -1, Motor::POSITION_SPEED,  // type, ratio, method
//           PID(25, 0.1, 10, 10, 1800),                // ppid
//           PID(400, 2, 0, 500, 30000),                // spid
//           true);                                     // use kf

// Friction wheel motors
// const PID fric_spid(6, 0.05, 2, 200, 16384);
// Motor FRICL(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
//             PID(), PID(fric_spid));         // ppid, spid
// Motor FRICR(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
//             PID(), PID(fric_spid));         // ppid, spid

// Stir motor
// Motor STIR(Motor::M2006, 36, Motor::POSITION_SPEED,  // type, ratio, method
//            PID(20, 0.1, 10, 10, 2500),               // ppid
//            PID(60, 0.1, 200, 1000, 10000));          // spid

/*
 * 【配置01 J0/J1】【TODO3-0：阅读已有配置，不用重写】
 * 输入：电机类型、传动比、控制方式、位置 PID、速度 PID、是否使用 KF 及其参数。
 * 输出：两个全局 Motor 对象；interface/robot.cpp 把它们传给全局 Arm arm。
 * 谁执行：C++ 启动时的全局对象初始化；这里不是每毫秒运行的代码。
 * 要调用谁：Motor::Motor、PID::PID、Motor::KFParam_t 构造函数。
 *
 * 第1步：joint_kf_param 设置过程噪声 Q 和测量噪声 R，不是 PID 增益。
 * 第2步：J0_ppid/J0_spid、J1_ppid/J1_spid 各是一套位置/速度环配置。
 * 第3步：用配置构造 J0/J1，true 表示使用滤波；PID 配置被复制进 Motor。
 * 第4步：ID 不由变量名字 J0/J1 决定，必须看后面的指针数组与 idConfig。
 *
 * PID 参数顺序：kp、ki、kd、i_max、out_max、d_filter_k。
 *   ppid 输出目标速度增量（deg/s），这里 out_max=1440。
 *   spid 输出驱动控制指令，这里 out_max=16384，不能当成 16384 A。
 *   i_max 限制的是累计误差 err_sum_；本工程 4 组 PID 都为 0，所以积分项被限制为零。
 *   因此 J0_spid 虽写了 ki=0.01，当前也没有非零积分输出；要结合 calc 看参数。
 *   d_filter_k 是微分项滤波系数；本算法按周期差分/累计，没有显式乘除 dt。
 *
 * ratio 当前按任务书为 J0=-1、J1=+1，负号统一反馈和输出的方向。
 * 实物传动链应与配置核对；不要只看 M3508 型号就擅自再除一次减速比。
 * Arm 的目标角保持统一正方向，不要为 J0 在上层再随意取负。
 */
// 两关节配置：CAN1 上 J1 ID=1、J0 ID=2；现有 PID 参数作为调试起点。
// Other motors
// 在线 KF Q 为 deg²、(deg/s)²，R 为 deg²
const Motor::KFParam_t joint_kf_param(5.f, 6e5f, 1.f);
const PID J0_spid(30,0.01,0,0,16384,1);
const PID J0_ppid(75,0,400,0,1440,1);
// J0 齿轮传动反向，用负传动比统一关节正方向；同时换算反馈和输出
Motor J0(Motor::M3508, -1, Motor::POSITION_SPEED,  // type, ratio, method
         J0_ppid,           // ppid
         J0_spid,           // spid
         true, joint_kf_param);  // 启用在线 KF

// J1 的构造顺序与 J0 一致；两关节负载不同，因此配置不必相同。
const PID J1_spid(12,0,50,0,16384,1);
const PID J1_ppid( 50,0,100,0,1440,1);
Motor J1(Motor::M3508, 1, Motor::POSITION_SPEED,
         J1_ppid,
         J1_spid,
         true, joint_kf_param);

/*
 * 【配置02 ID 表与驱动】【阅读已有实现】
 * can1_dji_motor[i] 存对象地址，对应 CAN1 的电机 ID=i+1，而不是关节编号。
 * &J1 放第0项 -> ID1；&J0 放第1项 -> ID2；nullptr 表示该 ID 没绑定电机。
 * dji_motor_driver 持有两张表，idConfig 读表登记 ID，收发也按表找对象。
 * C620 实际拨设/配置的 ID 要与表一致；写 &J1 不会替你修改电调内的 ID。
 * can2 表当前全空；MIT 驱动数组也为空，这次两关节不需要补 MIT 代码。
 */
// DJI Motor id config, M3508/M2006: 1~8, GM6020: 5~11
// DJI电机ID配置，M3508，M2006可配置范围为1~8，GM6020可配置范围为5~11
Motor* can1_dji_motor[11] = {
    &J1,   // id:1
    &J0,   // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    nullptr,    // id:5
    nullptr,     // id:6
    nullptr,  // id:7
    nullptr,      // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};
Motor* can2_dji_motor[11] = {
    nullptr,    // id:1
    nullptr,    // id:2
    nullptr,    // id:3
    nullptr,    // id:4
    nullptr,     // id:5
    nullptr,  // id:6
    nullptr,  // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

MITMotorDriver mit_motor_driver[] = {}; // slave_id必须小于0x0F

// 下面阈值仅供 allMotorsStopShutoff 判定切换模式；不是轨迹速度上限。
// 电机停转速度上限(dps)
const float motor_stop_rotate_speed_thres = 1200;

/*
 * 【01 allMotorsStopShutoff】【已实现，当前主线无调用，暂可跳过】
 * 输入：无形参；读各电机的连接表、模式和反馈速度。
 * 输出：void，修改各电机 mode_；不等待、不直接发送 CAN、不物理断电。
 * 谁调用：当前学生工程未发现调用点，是供其他上层使用的通用管理接口。
 * 要调用谁：fabs 取速度绝对值；真正的制动/零输出由后续 Motor::handle 执行。
 */
void allMotorsStopShutoff(void) {
  // 第1步：逐项看 CAN1/CAN2；跳过 nullptr 和 INIT。
  // 第2步：速度大于阈值则 STOP（本项目双环下目标速度为零），否则 POWEROFF。
  // 第3步：MIT 若有电机就直接 POWEROFF；当前为空，循环不会执行。
  // 注意：只调用一次且当时进入 STOP，不会自动再判断切到 POWEROFF；
  //       若使用此接口达到“先停再零输出”，上层要在后续周期继续调用。
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can1_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can1_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can1_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can2_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can2_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can2_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::POWEROFF;
  }
}

/*
 * 【02 allMotorsShutOff】【已实现，当前主线无调用】
 * 输入：无；输出：void，给已初始化的电机设置 POWEROFF。
 * 谁调用：供上层按需使用，当前机械臂停机主线走 Arm::disable。
 * 要调用谁：无；Motor::handle 随后按模式计算零控制量。
 * 第1步：检查指针非空且不是 INIT；第2步：设 POWEROFF；第3步：处理 MIT 表。
 * 不可把它理解为电源开关，也不会同步清除 Arm 的轨迹/使能状态。
 */
void allMotorsShutOff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::POWEROFF;
  }
}

/*
 * 【03 allMotorsStop】【已实现，当前主线无调用】
 * 输入：无；输出：void，给已初始化的电机设置 STOP。
 * 谁调用：供上层按需使用；要调用谁：无，后续 Motor::handle 响应该模式。
 * 第1步：遍历非空且非 INIT 的电机；第2步：设 STOP；第3步：处理 MIT 表。
 * 对本项目 POSITION_SPEED，STOP 用速度环追踪零转速，不是保持某个角度。
 * Arm 的“保持位置”应保留位置目标并继续 WORKING，两者作用不同。
 */
void allMotorsStop(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::STOP;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::STOP;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::STOP;
  }
}

/*
 * 【04 allMotorsOn】【已实现，当前主线无调用】
 * 输入：无；输出：void，给已初始化的电机设置 WORKING。
 * 谁调用：供上层按需使用；要调用谁：无。
 * 第1步：跳过空项和 INIT；第2步：设置 WORKING；第3步：处理 MIT 表。
 * 此函数只改 Motor 模式，不检查 Arm 是否标定，也不会从当前位置设置目标。
 * 本项目业务层应通过 arm.enable -> arm.handle 接管，不能用它替代 Arm::enable。
 */
void allMotorsOn(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::WORKING;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::WORKING;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::WORKING;
  }
}

/*
 * 【05 allMotorsInit】【TODO3-0：重点读】配置 ID、重置控制器并选反馈来源。
 * 输入：无；输出：void，初始化各电机软件状态。
 * 谁调用：interface/robot.cpp 的 controlTask，进入循环之前一次。
 * 要调用谁：DJIMotorDriver::idConfig、Motor::reset、Motor::setFdbSrc；
 *           MIT 分支当前为空。
 */
void allMotorsInit(void) {
  // 第1步：把指针表的通道和下标登记到各 Motor 的 ID 信息。
  dji_motor_driver.idConfig();
  // 第2步：遍历有效对象并 reset，清控制状态/PID 历史/滤波状态，不是机械回零。
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->reset();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->reset();
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->reset();
    i.setCmd(mitmotor::MOTOR_MODE);
  }
  // 第3步：选 KF 的角度/速度作为反馈数据源；传入地址，后续按地址读取最新值。
  // 注意：不要在每个周期重做 reset，否则历史状态一直丢失。
  // KF 输出仍使用 deg、deg/s；位置环和速度环都使用滤波反馈
  J0.setFdbSrc(&J0.kfAngle(), &J0.kfSpeed());
  J1.setFdbSrc(&J1.kfAngle(), &J1.kfSpeed());
}

/*
 * 【06 allMotorsHandle】【TODO3-0：重点读】让每台电机执行一次控制计算。
 * 输入：无形参；Motor 内部已有 Arm 写入的目标和 CAN 更新的反馈。
 * 输出：void；各 Motor 更新反馈/滤波、计算控制量并保存 intensity_。
 * 谁调用：interface/robot.cpp 的 controlTask，在 controlLoop 之后，每周期一次。
 * 要调用谁：每台 Motor::handle；MIT 的 handle 仅在数组非空时执行。
 * 这里的实际任务名是 controlTask，不是旧模板注释中的 motorTask。
 */
void allMotorsHandle(void) {
  // 第1步：遍历 ID 表，nullptr 不处理，已绑定对象调用一次 handle。
  // 第2步：Motor::handle 先判断连接和 INIT，再整理反馈和运行相应控制。
  //        POSITION_SPEED：目标角 -> 位置 PID + 速度前馈 -> 目标速度；
  //                         目标速度 -> 速度 PID + 输出前馈 -> 控制量。
  // 第3步：控制量存入 Motor；本函数不做 CAN 发送，canTask 随后独立发送。
  // 学习 C++：指针->handle() 与对象.handle() 都在调用成员函数，写法由变量类型决定。
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->handle();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->handle();
    }
  }
  for (auto & i : mit_motor_driver) {
    // call motor::handle() through MITMotorDriver::handle()
    i.handle();
  }
}

/*
 * 【07 motorsCanRxMsgHandle】【阅读已有实现】把收到的 CAN 帧交给对应驱动。
 * 输入：hcan=哪条 CAN；rx_header=帧 ID/类型/长度等；rx_data=收到的数据地址。
 * 输出：void；驱动匹配后更新对应 Motor 的原始反馈及连接状态。
 * 谁调用：interface/callback.cpp 的 HAL_CAN_RxFifo0MsgPendingCallback。
 * 要调用谁：DJIMotorDriver::canRxMsgCheck/canRxMsgCallback；以及 MIT 对应方法。
 * 第1步：检查帧是否属于 DJI 电机；匹配才交给 DJI 解包。
 * 第2步：遍历 MIT 驱动并各自检查；当前数组为空，不执行此部分。
 * 第3步：驱动根据 ID 找到 J0/J1，解析角度/速度/电流等，并刷新连接状态。
 * 注意：这里处于接收中断调用链，不写等待到位、延时或轨迹规划。
 *       实际控制反馈还会在周期 Motor::handle 中更新，Arm 不应重新解包 CAN。
 */
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }
  for (auto & i : mit_motor_driver) {
    if (i.canRxMsgCheck(hcan, rx_header)) {
      i.canRxMsgCallback(hcan, rx_header, rx_data);
    }
  }
}

/*
 * 【TODO4-2 调试记录：最后再看，不是要求重写以上函数】
 * 第1步：先验证映射/方向/单位/标定；这些错误不能靠加大 PID 参数解决。
 * 第2步：观察两关节的目标角与反馈角、目标速度与反馈速度、intensity_。
 *        同一张比较曲线必须用同一单位；Arm rad 与 Motor deg 不能直接相减。
 * 第3步：确认是否长期输出限幅、是否明显振荡或滞后，再逐项调整参数并记录。
 *        初次验证选小幅、较慢的目标；每次只改一类参数，便于判断效果。
 * 第4步：位置通路正确后，再接速度前馈/加速度前馈或比较滤波参数的影响。
 *        速度前馈由 Arm 提供；加速度前馈的系数在 interface/robot.cpp，
 *        不能把 rad/s² 直接当电流指令。当前该系数为零。
 * 不要为追踪误差去改 CAN ID、控制周期或重复调用 Motor::handle。
 */
