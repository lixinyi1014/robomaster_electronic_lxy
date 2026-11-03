#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H
#include <cmath>
struct CartesianReference {
  float x{}, y{};      // m
  float dx{}, dy{};    // m/s
  float ddx{}, ddy{};  // m/s²
};

// 同向伸直为零位，q1 相对第一杆
class ArmKinematics {
 public:
  static constexpr float l0 = 0.045f;
  static constexpr float l1 = 0.115f;
  static constexpr float min_sin_q1 = 0.01f;  // 雅可比奇异阈值

  static CartesianReference forward(const float q[2]);
  static bool lineReachable(const CartesianReference& start,
                            const CartesianReference& end);
  // elbow 为 +1/-1；q 展开到 previous_q 附近的圈数
  static bool inverse(const CartesianReference& point,
                      const float previous_q[2], int elbow, float q[2]);
  static bool derivatives(const CartesianReference& point, const float q[2],
                          float dq[2], float ddq[2]);
private:
  //lxy自定义::unwrap
  static float unwrap(float angle, float previous) {
    constexpr float two_pi = 6.283185307f;
    return angle + two_pi * std::round((previous - angle) / two_pi);
  }

  static float elbowCos(float radius_squared) {
    // cos q1 = (r^2 - l0^2 - l1^2) / 2 l0 l1
    return (radius_squared - ArmKinematics::l0 * ArmKinematics::l0 -
            ArmKinematics::l1 * ArmKinematics::l1) /
           (2.0f * ArmKinematics::l0 * ArmKinematics::l1);
  }

  static bool regularRadius(float radius_squared) {
    const float c = elbowCos(radius_squared);
    // 伸直、折叠或不可达时 1-c^2 太小，雅可比不能求逆
    return 1.0f - c * c > ArmKinematics::min_sin_q1 * ArmKinematics::min_sin_q1;
  }
};

#endif  // DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H
