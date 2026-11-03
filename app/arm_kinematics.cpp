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

#include "arm_kinematics.h"
#include <cmath>
/*
 * arm_kinematics：关节角 <-> 末端坐标，不让电机实际运动。
 * q[0]：第一杆相对 +X；q[1]：第二杆相对第一杆，逆时针为正，单位 rad。
 * 第二杆在世界平面的角度是 q[0]+q[1]，不能直接拿 q[1] 投影。
 * 末端坐标用 m；三角函数吃 rad，不是 deg。
 *
 * 先读 arm_kinematics.h：CartesianReference 是含六个 float 的结构体，
 * forward 返回它；inverse/derivatives 返回 bool，并用数组参数带回计算结果。
 * 无名参数只是模板占位，实现时按 .h 补名字，不需要改变接口类型。
 * 实现会用到 <cmath> 中 std::sin/cos/atan2/sqrt/round/fabs/fmin/fmax，
 * 本次未添加 include；需要使用时你再补。atan2 的顺序是 (y, x)。
 *
 * 尺寸入口：l0、l1 在 arm_kinematics.h，当前学生版为 0.045 m、0.115 m。
 * 这是当前代码值，不是重新测量结论；开始实物验证前应和两转轴/末端中心距核对。
 * 查调用：arm.cpp 的 trajStart/trajectoryPlanner；control.cpp 的 A 点预备动作。
 * 查数学：两杆投影、余弦定理、二阶矩阵求逆
 *
 * TODO2-2 的可选小助手（学生模板尚未定义，不会自动存在）：
 *   elbowCos(r2)：返回 (r2-l0*l0-l1*l1)/(2*l0*l1)，即 cos(q1)。
 *   regularRadius(r2)：先求 c，再判断 1-c*c > min_sin_q1*min_sin_q1。
 *     同时排除工作空间外与过分伸直/折叠，避免 sqrt 负数或雅可比除零。
 *   unwrap(angle, previous)：angle + 2*pi*round((previous-angle)/(2*pi))。
 *     从几何等价角里选最接近上一周期角度的一个，保留累计圈数。
 * 可以在本 .cpp 的匿名 namespace 中定义这些助手，或在下面函数内直接写公式。
 * 文件内助手若在类外，访问常量要写 ArmKinematics::l0/l1/min_sin_q1。
 */

/*
 * 【01 forward】【TODO2-1】已知两个角，求末端在哪。
 * 输入：const float q[2]，两个关节角，rad；const 表示只读输入。
 * 返回：CartesianReference；此函数只计算 x/y，其余导数保持零。
 * 谁调用（补全后）：Arm::trajStart 用上一段 reference_.q 得到直线起点。
 * 要调用谁：std::cos、std::sin。
 */
CartesianReference ArmKinematics::forward(const float q[2]) {
  // 第1步：给数组形参加名 q；创建 CartesianReference point{}。
  CartesianReference point{};
  float a = q[0]; float b = q[1] + q[0];
  point.x = l0 * cos(a) + l1 * cos(b);
  point.y = l0 * sin(a) + l1 * sin(b);
  return point;
  // 第2步：求两个绝对角 a = q[0]、b = q[0] + q[1]。
  // 第3步：point.x = l0*cos(a) + l1*cos(b)，将两杆的 X 投影相加。
  // 第4步：point.y = l0*sin(a) + l1*sin(b)，将两杆的 Y 投影相加。
  // 第5步：return point，调用者用 CartesianReference 或 auto 接收，不能用 float。
  // 注意：正解可以计算伸直姿态；后面的逆解/导数会主动排除奇异附近姿态。
  return CartesianReference{};
}

/*
 * 【02 inverse】【TODO2-2】已知末端位置，求一组连续的关节角。
 * 输入：point（只用 x/y，m）、previous_q[2]（上一组累计角，rad）、
 *       elbow（+1 或 -1，选择 sin(q1) 的正负，即弯肘分支）。
 * 输出：q[2] 写入两个解，rad；返回 true 表示求解成功，false 表示拒绝。
 * 谁调用（补全后）：Arm::trajectoryPlanner；robotControl 的预备姿态计算。
 * 要调用谁：上述可选助手，以及 std::sqrt、std::atan2、std::round。
 * 调用者必须检查 bool；失败后不能继续使用输出数组下发目标。
 */
bool ArmKinematics::inverse(const CartesianReference& point,
                      const float previous_q[2], int elbow, float q[2]) {
  // 第1步：依次补参数名 point、previous_q、elbow、q。
  if (elbow != 1 || elbow != -1) return false;
  float r2 = point.x * point.x + point.y * point.y;
  float cos_q1 = (r2-l0*l0-l1*l1)/(2*l0*l1);
  if (1-cos_q1*cos_q1 <= min_sin_q1*min_sin_q1) return false;
  float sin_q1 = static_cast<float>(elbow * sqrt(1-cos_q1*cos_q1));
  float q1 = atan2(sin_q1, cos_q1);
  float q0 = atan2(point.y, point.x) - atan2(l1*sin_q1, l0+l1*cos_q1);
  q[0] = unwrap(q0, previous_q[0]);
  q[1] = unwrap(q1, previous_q[1]);
  return true;


  // 第2步：检查 elbow 是 +1 或 -1；计算 r2 = point.x² + point.y²。
  // 第3步：c = (r2-l0²-l1²)/(2*l0*l1)，来自余弦定理，c 就是 cos(q1)。
  // 第4步：要求 1-c*c > min_sin_q1²；不满足就返回 false。
  //        不可达点不要强行把 c 夹进 [-1,1] 后冒充成功；伸直/折叠也在此拒绝。
  // 第5步：s = elbow * sqrt(1-c*c)，然后 q1_raw = atan2(s,c)。
  // 第6步：q0_raw = atan2(point.y,point.x) - atan2(l1*s,l0+l1*c)。
  //        前一项是末端方向，后一项扣除第二杆造成的夹角。
  // 第7步：分别把 q0_raw、q1_raw 用 unwrap 展开到 previous_q[0/1] 附近。
  //        例如上一角为 179 度、新主值为 -179 度，应选 181 度而非倒转 358 度。
  //        这里只用度数帮助理解；实际运算用 rad，整圈为 2*pi。
  // 第8步：写入 q[0]、q[1]，返回 true；累计角限位交给 Arm::withinLimits。

  //          previous_q 传原角，应还原原角附近的解；再测负肘支和跨圈连续性。
 // return false;
}

/*
 * 【03 lineReachable】【TODO2-3】判断整条直线能否画，不能只检查两端。
 * 输入：start、end 两个 CartesianReference，只使用位置 x/y，单位 m。
 * 返回：整段都在有效半径范围内、避开伸直/折叠附近时为 true。
 * 谁调用（补全后）：Arm::trajStart，在 trajectory_.start 之前。
 * 要调用谁：可选 regularRadius，以及 std::fmin/std::fmax 做区间限制。
 * 范围说明：这里只检查平面运动学和奇异距离，不检查机械碰撞、线缆或速度限制。
 */
bool ArmKinematics::lineReachable(const CartesianReference& start,
                                const CartesianReference& end) {
  // 第1步：补参数名 start、end；计算 dx=end.x-start.x、dy=end.y-start.y。
  float dx = end.x - start.x;
  float dy = end.y - start.y;
  float length2 = dx * dx + dy * dy;
  float u = 0.0f;
  if (length2 <= 0.0f) return false;
  u = -(start.x * dx + start.y * dy) / length2;
  u = std::fmax(0.0f,std::fmin(1.0f,u));
  float closest_x = start.x+u*dx;
  float closest_y = start.y+u*dy;
  float start_length = start.x*start.x + start.y*start.y;
  float end_length = end.x*end.x + end.y*end.y;
  float closest_length = closest_x*closest_x + closest_y*closest_y;
  return regularRadius(start_length)&&regularRadius(end_length)&&regularRadius(closest_length);
  // 第2步：length2 = dx*dx + dy*dy，这是线段长度的平方。
  // 第3步：先令 u=0；length2 > 0 时，计算
  //        u = -(start.x*dx + start.y*dy) / length2，再将 u 限制到 [0,1]。
  //        u 是“离原点最近的点”在线段上的比例，限制后不会落到延长线上。
  //        零长度线段不能做除法；保留 u=0，就只检查同一个点。
  // 第4步：closest_x=start.x+u*dx，closest_y=start.y+u*dy。
  // 第5步：计算 start、end、closest 三点各自的 x*x+y*y。
  // 第6步：三者都满足 regularRadius 才返回 true，否则返回 false。
  // 为什么三个点够用：线段上离原点最远处在某个端点，最近处是上述投影点。
  // 有效工作空间是环形；两个端点可达，也可能中间穿过不可达的内圈。
  //return false;
}

/*
 * 【04 derivatives】【TODO2-4】把末端速度/加速度换成两个关节的导数。
 * 输入：point.dx/dy（m/s）、point.ddx/ddy（m/s²），q[2]（rad）。
 * 输出：dq[2]（rad/s）、ddq[2]（rad/s²）；bool 表示换算是否成功。
 * 谁调用（补全后）：Arm::trajectoryPlanner，在 inverse 成功以后。
 * 要调用谁：std::sin/cos/fabs；使用 2x2 雅可比矩阵的显式公式即可。
 * 学习分两次：先看懂“位置逆解之后，还要换算导数”，再推导下面公式。
 * 当前 Arm 的规划设计会检查本函数返回值；一直返回 false 会中断每段直线。
 */
bool ArmKinematics::derivatives(const CartesianReference& point, const float q[2],
                          float dq[2], float ddq[2]) {
  // 第1步：补名 point、q、dq、ddq；检查 abs(sin(q[1])) > min_sin_q1。
  if (abs(sin(q[1])) <= min_sin_q1) return false;
  float s0 = sin(q[0]); float c0 = cos(q[0]);
  float s01 = sin(q[0]+q[1]); float c01 = cos(q[0]+q[1]);
  float j00 =-l0*s0-l1*s01; float j01=-l1*s01;
  float j10= l0*c0+l1*c01; float j11= l1*c01;
  float det=l0*l1*sin(q[1]);

  dq[0]=(j11*point.dx-j01*point.dy)/det;
  dq[1]=(-j10*point.dx+j00*point.dy)/det;
  const float speed01 = dq[0] + dq[1];
  const float ax =point.ddx + l0 *c0*dq[0]*dq[0] + l1*c01*speed01*speed01;
  const float ay =point.ddy+ l0 * s0 * dq[0] * dq[0]+ l1 * s01 * speed01 * speed01;
  ddq[0] = (j11 * ax - j01 * ay) / det;
  ddq[1] = (-j10 * ax + j00 * ay) / det;
  //        不满足返回 false，因为下面矩阵不可可靠求逆。
  // 第2步：计算 s0=sin(q0)、c0=cos(q0)、s01=sin(q0+q1)、c01=cos(q0+q1)。
  // 第3步：建立雅可比各元素：
  //        j00=-l0*s0-l1*s01；j01=-l1*s01；
  //        j10= l0*c0+l1*c01；j11= l1*c01；
  //        det=l0*l1*sin(q1)。这里 point 的速度 = J * 关节速度。
  // 第4步：用 J 的逆求速度：
  //        dq[0]=(j11*point.dx-j01*point.dy)/det；
  //        dq[1]=(-j10*point.dx+j00*point.dy)/det。
  // 第5步：令 v01=dq[0]+dq[1]，计算消除 J_dot*dq 后的加速度：
  //        ax=point.ddx+l0*c0*dq[0]²+l1*c01*v01²；
  //        ay=point.ddy+l0*s0*dq[0]²+l1*s01*v01²。
  // 第6步：ddq[0]=(j11*ax-j01*ay)/det；
  //        ddq[1]=(-j10*ax+j00*ay)/det；最后返回 true。
  // 注意：平方用乘法写，不要把数学的 ^ 当作 C++ 乘方运算。
  // 小检查：非奇异 q 下，point 的四个导数全零时，dq/ddq 也应全零。
  //          计算得到 dq 后，用 J*dq 应能还原 point.dx/dy。
  //
  // 初学位置验证阶段：可先完成第1步，并明确将 dq/ddq 四个元素置零后返回 true，
  // 这是暂不使用导数前馈的接口实现，不是已经完成上述换算。
  // 进入速度/精度调试阶段，再补第2~6步；不要只改成 true 而留下未赋值的数组。
  return true;
}
