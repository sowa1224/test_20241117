#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>

int main() {
    using namespace Eigen;

    // 初期設定
    int jointnum = 8; // ベクトルのサイズ
    VectorXd defaultpose = VectorXd(jointnum);
    Quaterniond q1(0, 0, 0, 1); // 初期姿勢

    // ロール、ピッチ、ヨー角度を指定（ラジアン）
    double offsetRoll = 10 * M_PI / 180.0;  // Roll: 30度
    double offsetPitch = 0 * M_PI / 180.0; // Pitch: 45度
    double offsetYaw = 0 * M_PI / 180.0;   // Yaw: 20度

    // 各軸の回転行列を作成
    Matrix3d Rroll, Rpitch, Ryaw;

    // Roll回転 (X軸回り)
    Rroll << 1, 0, 0,
             0, cos(offsetRoll), -sin(offsetRoll),
             0, sin(offsetRoll), cos(offsetRoll);

    // Pitch回転 (Y軸回り)
    Rpitch << cos(offsetPitch), 0, sin(offsetPitch),
              0, 1, 0,
              -sin(offsetPitch), 0, cos(offsetPitch);

    // Yaw回転 (Z軸回り)
    Ryaw << cos(offsetYaw), -sin(offsetYaw), 0,
            sin(offsetYaw), cos(offsetYaw), 0,
            0, 0, 1;

    // 合成された回転行列を計算（Roll -> Pitch -> Yawの順序）
    Matrix3d Rcombined = Ryaw * Rpitch * Rroll;

    // 合成された回転行列をクオータニオンに変換
    Quaterniond q2(Rcombined);

    // 初期姿勢q1と合成
    Quaterniond q3 = q1 * q2;

    // defaultposeにセット（例として位置とクオータニオンを入れる）
    defaultpose << 0.1, 0.4, -0.1, 0.33, q3.x(), q3.y(), q3.z(), q3.w();

    // 結果を表示
    std::cout << "Default Pose(qx, qy, qz, qw): " << q1.x() << ", " << q1.y() << ", " << q1.z() << ", " << q1.w() << std::endl;
    std::cout << "Quaternion (qx, qy, qz, qw): " << q3.x() << ", " << q3.y() << ", " << q3.z() << ", " << q3.w() << std::endl;

    return 0;
}