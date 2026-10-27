//
// Created by NoSignal on 2026/10/16.
//

/*
 * arm.cpp 学习导航
 *
 * 本文件仍是待完成的学生模板。下面只添加学习注释，没有补写实现。
 * “实现步骤”表示你接下来可以按顺序写的语句，不表示当前已完成。
 * 现有 return false 是占位返回；空函数现在不做事。
 *
 * 先认清三个量：
 *   state_     = 实际反馈：“机械臂现在在哪里、转得多快”。
 *   reference_ = 控制目标：“希望机械臂在哪里、转得多快”。
 *   mode_      = Arm 层是否使能；Motor::mode_ 是每台电机自己的运行状态。
 *   calibrated_= 是否已经按已知机械姿态标定；使能和标定是两件事。
 *
 * 单位：Arm 用 rad、rad/s、rad/s²；末端坐标用 m；时间用 s。
 *       Motor 的角度和角速度接口用 deg、deg/s，不能混用。
 *       q[0] 是 J0 角度；q[1] 是 J1 相对第一连杆的角度。
 *
 * 补全后的调用主线（每个控制周期只推进一步）：
 *   controlTask -> controlLoop
 *     -> robotControl：按遥控器操作调用标定/使能/启动轨迹等接口。
 *     -> arm.handle：读反馈 -> 推进轨迹 -> 写入 Motor 目标。
 *   controlTask -> allMotorsHandle：之后才由 Motor::handle 计算 PID。
 *   canTask -> canTxMonitor：另一个任务负责把电机控制量发到 CAN。
 *
 * 当前已经存在的调用：robot.cpp 创建 arm；controlLoop 调 handle；
 * robotReset 调 disable。学生版 robotControl 还是空的，其他上层调用待补。
 * 下文标为“补全后”的内部调用关系，参考任务书、接口和完整参考工程。
 *
 * 推荐实现顺序：
 *   feedbackReady / withinLimits / updateFeedback
 *   -> calibrate / enable / disable / applyReference / setReference
 *   -> addJointReference / handle / writeMotorCommands
 *   -> trajectoryReady / trajAbort / trajStartJoint
 *   -> trajStart / trajectoryPlanner（需要运动学和插值模块配合）。
 *
 * C++ 小提示：
 *   Arm:: 表示“这个函数属于 Arm 类”；末尾带 _ 的名字是成员变量。
 *   bool 返回成功/失败或条件是否成立；void 不返回值，但仍可修改成员。
 *   模板里的 float、const Reference& 没有参数名，当前只是占位。
 *   真正实现时可按 arm.h 给它们补参数名，本次没有修改这些声明。
 *
 * 后续实现可能需要的头文件（目前未添加）：
 *   base/common/math.h：math::deg2rad / rad2deg / limit。
 *   main.h：HAL_GetTick；FreeRTOS.h 和 task.h：临界区宏。
 *   <cmath>：std::sin 等数学函数。
 */
#include "arm.h"  // 引入 Arm 的类声明、成员变量和所依赖的类型。

#include "task.h"
#include "FreeRTOS.h"
#include "base\common\math.h"

/*
 * 【01 构造函数】把两台现有 Motor 对象交给 Arm 管理。
 * 输入：joint0/joint1 是电机对象的引用；limits 是角度上下限；
 *       feedforward 是加速度前馈系数。引用不是再复制一台电机。
 * 输出：没有返回类型；创建并初始化一个 Arm 对象。
 * 谁调用：interface/robot.cpp 中的 Arm arm(J0, J1, ...)。
 * 要调用谁：这里不需要启动外设，也不需要发 CAN 或计算 PID。
 * 这段初始化列表已经写好；mode_ 等默认值在 arm.h 中定义。
 */
Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits),
      feedforward_(feedforward) {}
// 上面第一行：指定构造函数接收哪两个 Motor 和哪套限位。
// 第二行：接收前馈系数；const 引用表示通过该参数读取配置。
// 第三行：让成员引用绑定到电机，并把限位配置复制到 limits_。
// 第四行：复制前馈配置；{} 是空函数体，初始化工作已在冒号后完成。

/*
 * 【02 feedbackReady】判断“两台电机的反馈都能使用了吗”。
 * 输入：没有形参；读取 joint0_、joint1_ 的连接状态和 Motor::mode_。
 * 返回：两台都在线且都已离开 Motor::INIT 时为 true，否则 false。
 * 谁调用（补全后）：calibrate、enable、trajectoryReady、handle。
 * 要调用谁：两台电机的 connect_.check()。
 */
//lxy: 这里只检查电机的反馈是否允许被使用，也即是能收到且电机的状态是可以的
bool Arm::feedbackReady() {
  // 第1步：检查 joint0_.connect_.check()，确认 J0 没有反馈超时。
  // 第2步：对 J1 做相同检查；只收到一台电机的反馈还不够。
  if (!joint0_.connect_.check() || !joint1_.connect_.check()) return false;
  // 第3步：检查两台 Motor 的 mode_ 都不是 INIT，避免使用初始化中的角度。
  if ((joint0_.mode_ != Motor::Mode_e::INIT)||(joint1_.mode_ != Motor::Mode_e::INIT)) return false;
  // 第4步：用逻辑与 && 合并这些条件，并返回判断结果。
  // 注意：这里不检查 calibrated_。能收到反馈不等于已经知道机械零位。
  //return false;  // 原模板占位；目前无论状态如何都返回 false。
  return true;
}

/*
 * 【03 withinLimits】检查一对目标角是否在允许范围内。
 * 输入：一个含两个 float 的数组，按 arm.h 可命名为 q，单位 rad。
 * 返回：两个关节都在闭区间 [q_min, q_max] 内时为 true。
 * 谁调用（补全后）：calibrate、applyReference、trajStart、trajStartJoint。
 * 要调用谁：不需要其他函数，只读取 limits_ 并比较数值。
 * 末尾 const 表示此函数不修改 Arm 的成员状态；不会把越界角自动夹回。
 */
bool Arm::withinLimits(const float q[2]) const {
  // 第1步：比较 q[0] 与 limits_.q_min[0]、limits_.q_max[0]。
  if (q[0] < limits_.q_min[0] || q[0] > limits_.q_max[0]) return false;
  // 第2步：比较 q[1] 与 limits_.q_min[1]、limits_.q_max[1]。
  if (q[1] < limits_.q_min[1] || q[1] > limits_.q_max[1]) return false;
  // 第3步：两个角都满足上下限才返回 true，任意一个越界就返回 false。
  // 注意：检查的是累计角度，不要先把它们归一化到正负 180 度。
  return true;  // 原模板占位。
}

/*
 * 【04 calibrate】把“当前机械姿态”登记为输入的已知角度。
 * 输入：两个 float 依次为 q0_rad、q1_rad，单位 rad。
 *       两杆同向沿 +X 摆直时可传 0、0；这不是命令电机转到零点。
 * 返回：成功完成标定为 true；已使能、角度越限或反馈无效为 false。
 * 修改：电机控制反馈的零位、calibrated_、state_、reference_ 和轨迹状态。
 * 谁调用（待补上层）：robotControl 的“左中右中”标定分支。
 * 要调用谁：withinLimits、feedbackReady、Motor::resetFeedbackAngle、
 *           math::rad2deg、trajAbort、updateFeedback。
 */
bool Arm::calibrate(float q0, float q1) {
  // 第1步：把两个输入角组成 q[2]；检查 mode_ 必须为 Disabled。
  const float q[2] = {q0, q1};
  if (mode_ != Mode::Disabled) return false;
  // 第2步：用 withinLimits(q) 检查已知姿态，失败就返回 false。
  if (!withinLimits(q)) return false;
  // 第3步：进入短临界区 taskENTER_CRITICAL，防止 CAN 回调同时改反馈。
  taskENTER_CRITICAL();
  // 第4步：在临界区内检查 feedbackReady；失败时先退出临界区再返回。
  if (!feedbackReady()) taskEXIT_CRITICAL();
  // 第5步：把两个 rad 输入转成 deg，分别调用两台 Motor 的 resetFeedbackAngle。
  joint0_.resetFeedbackAngle(math::rad2deg(q0));
  joint1_.resetFeedbackAngle(math::rad2deg(q1));
  // 第6步：设置 calibrated_ = true，再 taskEXIT_CRITICAL。
  calibrated_ = true;
  taskEXIT_CRITICAL();
  // 第7步：trajAbort 取消旧轨迹；updateFeedback 读取修改零位后的状态。
  trajAbort();
  updateFeedback();
  // 第8步：把 reference_ 清成 Reference{}，再将 state_.q 复制到 reference_.q。
  //        这样目标位置等于标定后的当前位置，目标速度/加速度都是零。
  reference_ = Reference{};
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  // 第9步：返回 true。临界区必须成对退出，不要把耗时计算放在里面。
  return true;
}

/*
 * 【05 enable】允许控制，并从当前位置平稳接管。
 * 输入：没有形参；需要 calibrated_ 为真且 feedbackReady() 为真。
 * 返回：已使能或本次使能成功为 true；条件不满足为 false。
 * 修改：reference_、两台电机的 PID 状态、Arm 的 mode_。
 * 谁调用（待补上层）：robotControl 的手动分支和自动轨迹启动分支。
 * 要调用谁：feedbackReady、updateFeedback、两台 Motor 的 ppid_/spid_.reset。
 */
bool Arm::enable() {
  // 第1步：未标定或反馈无效就返回 false，避免带着错误零位运动。
  if (!calibrated_||!feedbackReady()) return false;
  // 第2步：如果已经是 Mode::Joint，直接返回 true。
  if (mode_ == Mode::Joint) return true;
  //        不要每毫秒重复清 PID 或把正在执行的目标改回当前反馈。
  // 第3步：updateFeedback 取得当前位置；reference_ 清零后复制 state_.q。
  updateFeedback();
  reference_ = Reference{};
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  // 第4步：清除 J0、J1 的位置 PID 和速度 PID 历史状态。
  joint0_.ppid_.reset();
  joint1_.ppid_.reset();
  joint0_.spid_.reset();
  joint1_.spid_.reset();
  // 第5步：设置 Arm 的 mode_ = Mode::Joint，并返回 true。
  mode_ = Mode::Joint;
  // 注意：真正的目标下发由后续 handle -> writeMotorCommands 完成。
  return true;
}

/*
 * 【06 disable】取消运动并要求两台电机输出零控制量。
 * 输入：没有形参；输出：void，通过成员变量改变后续电机控制行为。
 * 谁调用：现有 robotReset；补全后的标定分支和 handle 掉线分支。
 * 要调用谁：trajAbort；并修改两台 Motor 的模式和控制输出。
 * 这里的失能是软件零输出，不会物理切断电调供电。
 */
void Arm::disable() {
  // 第1步：trajAbort 取消运行中的轨迹，清掉速度和加速度前馈。
  trajAbort();
  // 第2步：把 Arm 的 mode_ 设为 Disabled。
  mode_ = Mode::Disabled;
  // 第3步：两台 Motor 若不是 INIT，就把各自 mode_ 设为 POWEROFF。
  if (joint0_.mode_ != Motor::Mode_e::INIT) joint0_.mode_ = Motor::Mode_e::POWEROFF;
  if (joint1_.mode_ != Motor::Mode_e::INIT) joint1_.mode_ = Motor::Mode_e::POWEROFF;
  //        INIT 要交给 Motor::handle 完成首次反馈初始化，不能强行跳过。
  // 第4步：两台 Motor 的 intensity_ 和 intensity_float_ 都置零。
  joint0_.intensity_ = 0.0f;
  joint0_.intensity_float_ = 0.0f;
  joint1_.intensity_ = 0.0f;
  joint1_.intensity_float_ = 0.0f;
  //        前者用于 CAN 发送，后者是 PID 计算的浮点控制量。
  // 注意：普通失能可以保留标定；因掉线失能时，由 handle 清除 calibrated_。
  // 当前空函数什么也不做；此处没有隐含的自动停机操作。
}

/*
 * 【07 setReference】供外部直接提交一组关节位置/速度/加速度目标。
 * 输入：const Reference&，包含 q[2]、dq[2]、ddq[2]，单位见文件顶部。
 * 返回：没有轨迹占用且目标被接受时为 true，否则 false。
 * 谁调用：供上层按需调用，当前学生版尚无调用点。
 * 要调用谁：trajRunning、applyReference。
 */
//lxy：这个看起来没有必要独立出来啊
//lxy：看懂了，也就是说不同的地方在调用，但命名是否应该更好一些？🤔
bool Arm::setReference(const Reference& reference) {
  // 第1步：检查 trajRunning；正在自动运动时拒绝外部目标，返回 false。
  if (trajRunning()) return false;
  // 第2步：把输入交给 applyReference，由它统一检查限位和保存目标。
  return applyReference(reference);
  // 第3步：将 applyReference 的返回值作为本函数的结果。
  // 注意：这只是提交目标，不是立即运动到位，也不负责使能机械臂。
}

/*
 * 【08 applyReference】内部统一的“检查并保存目标”入口。
 * 输入：一组 Reference；返回：角度合法并已保存为 true，否则 false。
 * 修改：成功时更新 reference_，失败时保留上一组目标。
 * 谁调用（补全后）：setReference、addJointReference、trajectoryPlanner。
 * 要调用谁：withinLimits。
 */
bool Arm::applyReference(const Reference& reference) {
  // 第1步：检查输入 reference.q 是否满足 withinLimits。
  if (!withinLimits(reference.q)) return false;
  // 第2步：不满足则返回 false，不要先覆盖 reference_ 再检查。
  // 第3步：合法时把整组输入复制给 reference_，随后返回 true。
  reference_ = reference;
  return true;
  // 与 setReference 的区别：内部轨迹也要用本函数，所以这里不拒绝“轨迹运行中”。
  // 注意：这个简化检查只覆盖角度范围，不等于已检查机械碰撞和速度上限。

}

/*
 * 【09 addJointReference】把手动输入的关节速度积成下一个位置目标。
 * 输入：三个 float 依次为 speed0、speed1（rad/s）、dt_s（s）。
 *       arm.h 给 dt_s 默认值 0.001，即正常控制周期 1 ms。
 * 返回：状态允许且目标已接受时为 true，否则 false。
 * 谁调用（待补上层）：robotControl 的“左上右上”摇杆控制分支。
 * 要调用谁：trajRunning、math::limit、applyReference。
 */
bool Arm::addJointReference(float, float, float) {
  // 第1步：若不是 Mode::Joint，或已有轨迹运行，返回 false。
  // 第2步：把 reference_ 复制为局部 next；用两个速度分别处理两个关节。
  // 第3步：计算 next_q = 原目标角 + 输入角速度 * dt_s。
  //        用原目标积分可保持目标连续，不要每次用滞后的实际反馈重新起算。
  // 第4步：用 math::limit 将 next_q 限制在对应角度上下限内。
  // 第5步：把输入角速度写入 next.dq，作为速度前馈。
  // 第6步：若已碰到限位且仍向外运动，则把该关节 next.dq 置零。
  // 第7步：手动分支没有规划加速度时，将 next.ddq 置零。
  // 第8步：调用 applyReference(next) 并返回结果；不要在这里发送 CAN。
  return false;  // 原模板占位。
}

/*
 * 【10 trajectoryReady】检查现在能否启动一条新轨迹。
 * 输入：没有形参；返回：已使能、已标定、反馈有效且没有轨迹运行时为 true。
 * 谁调用（补全后）：trajStart、trajStartJoint。
 * 要调用谁：feedbackReady、trajRunning。
 */
bool Arm::trajectoryReady() {
  // 第1步：检查 mode_ == Mode::Joint。
  // 第2步：检查 calibrated_ 和 feedbackReady()。
  // 第3步：检查 !trajRunning()，防止新轨迹覆盖尚未结束的轨迹。
  // 第4步：返回这些条件的逻辑与；这里只检查，不自动使能或自动标定。
  return false;  // 原模板占位。
}

/*
 * 【11 trajStart】启动一条“末端沿直线走到指定点”的轨迹。
 * 输入：三个 float 依次为 x_m、y_m（终点坐标，m）、duration_s（用时，s）。
 * 返回：轨迹成功登记为 true；未准备好、不可达或时间非法为 false。
 * 谁调用（待补上层）：robotControl 在方形各边的衔接处。
 * 要调用谁：trajectoryReady、ArmKinematics::forward / lineReachable、
 *           withinLimits、trajectory_.start、std::sin、HAL_GetTick。
 */
bool Arm::trajStart(float, float, float) {
  // 第1步：trajectoryReady 不满足就返回 false。
  // 第2步：创建末端终点 end，写入 x_m、y_m。
  // 第3步：用 forward(reference_.q) 计算起点，取的是上一段规划目标。
  //        不能换成 state_.q，否则跟踪误差会使下一条边的规划起点偏离顶点。
  // 第4步：检查起始关节角 withinLimits；检查整条直线 lineReachable。
  // 第5步：调用 trajectory_.start(start, end, duration_s)，失败则返回 false。
  // 第6步：根据 sin(reference_.q[1]) 的正负保存 trajectory_elbow_ = +1 或 -1。
  //        这决定后续逆解保持哪种弯肘姿态，累计圈数仍由 inverse 保留。
  // 第7步：用 HAL_GetTick 保存 trajectory_start_tick_（单位是 ms）。
  // 第8步：设置 trajectory_type_ = Cartesian，trajectory_finished_ = false。
  // 第9步：返回 true。此时只是“已启动”，并不是已经走到终点。
  // 后续每次 handle 会推进一点；这里不能用 while 或延时等它走完。
  return false;  // 原模板占位。
}

/*
 * 【12 trajStartJoint】启动两个关节同步的点到点运动，用来移到预备姿态。
 * 输入：q0_rad、q1_rad（累计目标角，rad）、duration_s（s）。
 * 返回：成功登记轨迹为 true；状态/限位/时间不满足为 false。
 * 谁调用（待补上层）：robotControl 求出 A 点逆解后，启动约 2 s 的预备动作。
 * 要调用谁：trajectoryReady、withinLimits、updateFeedback、HAL_GetTick。
 */
bool Arm::trajStartJoint(float, float, float) {
  // 第1步：把两个输入角组成 end[2]。
  // 第2步：检查 trajectoryReady、终点限位和 duration_s > 0。
  // 第3步：updateFeedback 读取实际姿态，再检查起始反馈角没有越限。
  // 第4步：reference_ 清零；保存 state_.q 为 joint_trajectory_start_。
  // 第5步：保存 end 为 joint_trajectory_end_，并以当前 state_.q 初始化位置目标。
  // 第6步：保存 joint_trajectory_duration_ 和 trajectory_start_tick_。
  // 第7步：设置 trajectory_type_ = Joint，trajectory_finished_ = false。
  // 第8步：返回 true。后续由 trajectoryPlanner 对两个角使用同一时间插值。
  // 注意：本函数从实际反馈起步，适合预备运动；它不保证末端走直线。
  // 从伸直零位出发时，关节插值也不需要在该奇异起点求逆雅可比。
  return false;  // 原模板占位。
}

/*
 * 【13 trajAbort】取消轨迹推进，并保留最后的位置目标。
 * 输入：没有形参；输出：void，修改轨迹状态和 reference_ 的导数项。
 * 谁调用：补全后的 disable、calibrate、trajectoryPlanner 失败分支，
 *         以及 robotControl 的切挡/中止分支。
 * 要调用谁：trajectory_.abort。
 */
void Arm::trajAbort() {
  // 第1步：trajectory_.abort()，清除末端插值器内部的 running_。
  // 第2步：设置 trajectory_type_ = None，终止 Arm 层的两种轨迹。
  // 第3步：设置 trajectory_finished_ = false，表明这是取消，不是正常完成。
  // 第4步：两个关节 reference_.dq、reference_.ddq 全部置零。
  // 第5步：保留 reference_.q，让已使能的电机保持最后的规划位置目标。
  // 注意：不要把“取消”写成“正常完成”，否则上层可能自动启动下一条边。
  // 注意：保持的是最后的目标，不一定等于实际反馈位置；取消也不等于失能。
}

/*
 * 【14 trajectoryPlanner】根据已经过去的时间，算出“本周期”的目标。
 * 输入：没有形参；读取轨迹类型、起始时间、轨迹参数和上一组 reference_。
 * 输出：void；更新 reference_，并在结束或失败时更新轨迹标志。
 * 谁调用（补全后）：handle，每个控制周期调用一次。
 * 要调用谁：trajRunning、HAL_GetTick、ArmTrajectory::interpolate、
 *           trajectory_.sample / running、ArmKinematics::inverse / derivatives、
 *           applyReference、trajAbort。
 */
void Arm::trajectoryPlanner() {
  // 第1步：没有轨迹运行就直接 return，保留原来的位置目标。
  // 第2步：elapsed_s = (HAL_GetTick() - trajectory_start_tick_) * 0.001f。
  //        时间差先用无符号计数计算，再换算成秒；不要把 ms 当 s。
  //
  // 第3步：如果轨迹类型是 Joint（预备运动）：
  //   3.1 创建局部 Reference next，两个关节使用同一个 elapsed_s 和总用时。
  //   3.2 分别调用 interpolate，填写 next.q、next.dq、next.ddq。
  //   3.3 applyReference(next) 失败则 trajAbort。
  //   3.4 成功且时间已到终点：类型设为 None，finished 设为 true。
  //   3.5 直接 return，不能继续执行下面的末端直线分支。
  //
  // 第4步：如果轨迹类型是 Cartesian（末端直线）：
  //   4.1 trajectory_.sample(elapsed_s) 得到末端位置、速度、加速度。
  //   4.2 inverse(point, reference_.q, trajectory_elbow_, next.q) 求关节角。
  //       上一组 reference_.q 用于保留角度连续性和累计圈数。
  //   4.3 derivatives(point, next.q, next.dq, next.ddq) 求关节速度/加速度。
  //   4.4 applyReference(next) 检查限位并保存目标。
  //   4.5 上述任意一步失败则 trajAbort；不要把无效计算结果下发。
  //   4.6 全部成功且插值器已结束：类型设为 None，finished 设为 true。
  //
  // 注意：先成功输出最终目标，再标记完成，防止漏掉最后一个采样点。
  // finished 只表示规划参考结束，不代表实际电机已经达到误差要求。
  // 若沿用这个调用链，学生版 derivatives 不能一直返回 false，否则会取消轨迹。
  // 初学先不做前馈时，需要显式输出零导数并合理返回成功；不能留未初始化值。
}

/*
 * 【15 updateFeedback】把两台 Motor 的反馈转换为 Arm 使用的单位。
 * 输入：没有形参；读取两台电机 realAngle()、realSpeed()。
 * 输出：void，写入 state_.q[0/1] 和 state_.dq[0/1]。
 * 谁调用（补全后）：handle、calibrate、enable、trajStartJoint。
 * 要调用谁：Motor::realAngle / realSpeed、math::deg2rad，以及临界区宏。
 */
void Arm::updateFeedback() {
  // 第1步：准备局部 q[2]、dq[2]，暂存 Motor 的 deg 和 deg/s。
  // 第2步：进入短临界区，再读取 J0、J1 的角度和角速度到局部数组。
  // 第3步：退出临界区，使 CAN 中断可以继续更新反馈。
  //        临界区避免复制到一半被中断改写；它不保证两台电机物理同时采样。
  // 第4步：对每个关节做 deg2rad，分别写入 state_.q、state_.dq。
  // 注意：Motor 层已按 ratio_ 处理传动与方向，Arm 层不要再除一次减速比。
  // 注意：这里不重新读 CAN。CAN 回调先更新反馈，Motor::handle 再整理控制反馈。
  // 当前任务顺序下，realAngle/realSpeed 是最近一次 Motor::handle 整理的结果。
}

/*
 * 【16 handle】机械臂周期入口，把本文件的各个小步骤连起来。
 * 输入：没有形参；输出：void，更新状态、轨迹和两台电机的软件目标。
 * 谁调用：app/control.cpp 的 controlLoop；正常按 1 ms 节拍调用。
 * 要调用谁：updateFeedback、feedbackReady、disable、trajectoryPlanner、
 *           writeMotorCommands。
 */
void Arm::handle() {
  // 第1步：updateFeedback，更新 Arm 保存的实际状态。
  // 第2步：若 feedbackReady 为 false：
  //        清除 calibrated_，调用 disable，然后立即 return。
  //        掉线期间可能漏掉编码器圈数，重连后必须重新标定。
  // 第3步：如果 mode_ == Disabled，直接 return，不输出新的运动目标。
  // 第4步：trajectoryPlanner 推进当前轨迹；没有轨迹时它会自行返回。
  // 第5步：writeMotorCommands 将当前 reference_ 写给两台 Motor。
  // 注意：保持目标时也要正常走到第5步，不是只有轨迹运行时才调用它。
  // 注意：这里不要额外调用 Motor::handle 或发送 CAN，外层已有统一调度。
  // 当前空函数不执行上述步骤，需你按顺序补齐。
}

/*
 * 【17 writeMotorCommands】将 Arm 目标转换为 Motor 接口的角度单位。
 * 输入：没有形参；读取 reference_ 和 feedforward_。
 * 输出：void，修改两台 Motor 的目标角、速度前馈、输出前馈和运行模式。
 * 谁调用（补全后）：handle，要求已使能且反馈有效。
 * 要调用谁：math::rad2deg、两台 Motor 的 setAngleSpeed。
 */
void Arm::writeMotorCommands() {
  // 第1步：将两台 Motor 的 method_ 设为 POSITION_SPEED（位置+速度双环）。
  // 第2步：对 J0，把 reference_.q[0] 从 rad 转成 deg。
  // 第3步：把 reference_.dq[0] 从 rad/s 转成 deg/s，作为速度前馈。
  // 第4步：计算 feedforward_.k_acc[0] * reference_.ddq[0]，作为输出前馈。
  //        该系数把角加速度转换成控制指令；默认是零，尚未建立动力学模型。
  // 第5步：joint0_.setAngleSpeed(目标角, 速度前馈, 输出前馈)。
  // 第6步：对 J1 使用下标 1 做同样的转换和调用。
  // 第7步：将两台 Motor 的 mode_ 设为 WORKING，允许后续执行 PID。
  // 注意：setAngleSpeed 只保存目标，不等待到位，也不会直接发送 CAN。
  // 后续 allMotorsHandle -> Motor::handle 算控制指令，canTask 再统一发送。
  // 输出前馈/intensity_ 是驱动使用的控制指令值，不能直接当作安培。
}
