#pragma once

#include <Eigen/Geometry>

/**
 * @brief 红瓶插入任务的 T_OBJ 偏置（与三角块的 buildTcpOffset 同理，但针对插入）。
 *
 * 瓶子网格 bottle_red.obj / bottle_red.STL 的建模坐标系：
 *   原点在包围盒最小角 (0,0,0)，单位 mm；
 *   瓶轴 = +Z，瓶口在 z=130mm，瓶口中心在 (25, 25, 130)mm；
 *   瓶子中空、壁厚约 3~4mm，瓶口内径约 35mm。
 *
 * Tool_Link 是一根杆子：沿 Tool_Link 系 -X 方向伸出 80mm（尖头在 x=-0.08m），
 * 直径 20mm，正好能插入瓶口。
 *
 * 姿态约定：杆子竖直、尖头朝下。
 *   Tool_Link +X 朝上（= +Z_obj），则 -X（尖头）朝下插入瓶口。
 */
namespace bottle_offset {

// 瓶口中心在建模坐标系中的 (x, y) 与瓶口所在高度（单位：米）
constexpr double kOpeningX = 0.025;
constexpr double kOpeningY = 0.025;
constexpr double kRimZ = 0.130;

// 杆长 / 插入深度 / 上方停靠高度（单位：米）
constexpr double kRodLength = 0.080;
constexpr double kInsertDepth = 0.020;      // 尖头伸进瓶口 20mm
constexpr double kApproachStandoff = 0.030; // 尖头停在瓶口上方 30mm

// 杆子竖直、尖头朝下的姿态：
//   X_tcp = [0,0,1]（朝上） Y_tcp = [0,1,0]  Z_tcp = [-1,0,0]
inline Eigen::Isometry3d buildRodVerticalDown()
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  // 杆尖在 -X（尖头 x=-0.08），要让杆尖朝下（-Z_obj）插瓶，则 +X 朝上（+Z_obj）。
  // Eigen 列主序赋值：列 0=X_tcp=(0,0,1) 列 1=Y_tcp=(0,1,0) 列 2=Z_tcp=(-1,0,0)
  T.linear() << 0.0, 0.0, 1.0,
                0.0, 1.0, 0.0,
                -1.0, 0.0, 0.0;
  return T;
}

// 上方对齐偏置：杆子尖头停在瓶口正上方 kApproachStandoff 处
inline Eigen::Isometry3d approachOffset()
{
  Eigen::Isometry3d T = buildRodVerticalDown();
  T.translation() = Eigen::Vector3d(
      kOpeningX, kOpeningY, kRimZ + kApproachStandoff + kRodLength);
  return T;
}

// 插入到位偏置：杆子尖头伸进瓶口 kInsertDepth
inline Eigen::Isometry3d insertOffset()
{
  Eigen::Isometry3d T = buildRodVerticalDown();
  T.translation() = Eigen::Vector3d(
      kOpeningX, kOpeningY, kRimZ - kInsertDepth + kRodLength);
  return T;
}

}  // namespace bottle_offset
