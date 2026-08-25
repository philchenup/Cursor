#include "RlVectorToStdVector.h"

#include <vector>

// 用法示例：把 RL 关节角 / 笛卡尔增量转成 float 数组，便于发给控制器或日志
std::vector<float> JointsAsFloat(const rl::math::Vector& q)
{
    return rlVectorToStdVector(q);
}

std::vector<float> Vector3AsFloat(const rl::math::Vector3& p)
{
    return rlVectorToStdVector(p);
}
