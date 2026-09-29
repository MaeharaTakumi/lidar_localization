/*
 * ndt_initial_guess.c
 *
 * 差動二輪車両のオドメトリ (v, omega) から
 * NDT-matching の初期値となる LiDAR 姿勢を予測する。
 *
 * 前提: 車両は水平面を移動する（base_link の roll/pitch は 0）
 */

#include <math.h>
#include <stdio.h>

/* ===== パラメータ ===== */

/* 車両中心（base_link）から見た LiDAR の位置・姿勢（外部パラメータ） */
static const double X_OFFSET     = 0.80;   /* x位置 [m] */
static const double Y_OFFSET     = 0.00;   /* y位置 [m] */
static const double Z_OFFSET     = 1.20;   /* z位置 [m] */
static const double ROLL_OFFSET  = 0.00;   /* roll  [rad] */
static const double PITCH_OFFSET = 0.00;   /* pitch [rad] */
static const double YAW_OFFSET   = 0.00;   /* yaw   [rad] */

/* 数値パラメータ */
static const double DT              = 0.10;    /* 刻み時間 [s] */
static const double SINC_TAYLOR_TH  = 1.0e-4;  /* sinc をテイラー展開で評価する閾値 [rad] */

/* ===== 型定義 ===== */

/* 6自由度の位置姿勢。オイラー角は ZYX 順（R = Rz(yaw)*Ry(pitch)*Rx(roll)） */
typedef struct {
    double x;      /* [m]   */
    double y;      /* [m]   */
    double z;      /* [m]   */
    double roll;   /* [rad] */
    double pitch;  /* [rad] */
    double yaw;    /* [rad] */
} Pose6;

/* ===== 補助関数 ===== */

/*
 * sinc 関数 sin(a)/a を 0 除算なしで評価する
 *
 * 入力:
 *     a         : 引数 [rad]
 *     taylor_th : テイラー展開に切り替える閾値 [rad]
 * 出力:
 *     sin(a)/a の値（a=0 で 1.0）
 */
static double sinc(double a, double taylor_th)
{
    /* 微小域は sin(a)/a が 0/0 になるため 4 次までのテイラー展開で代用 */
    if (fabs(a) < taylor_th) {
        const double a2 = a * a;
        return 1.0 - a2 / 6.0 + a2 * a2 / 120.0;
    }
    return sin(a) / a;
}

/*
 * 角度を [-pi, pi) に正規化する
 *
 * 入力:
 *     angle : 角度 [rad]
 * 出力:
 *     正規化された角度 [rad]
 */
static double normalize_angle(double angle)
{
    return atan2(sin(angle), cos(angle));
}

/* ===== 主要関数 ===== */

/*
 * 差動二輪の 1 ステップ並進増分を円弧の厳密積分で求める（車体座標系）
 *
 * 入力:
 *     v         : 車両中心の並進速度 [m/s]
 *     omega     : 車両中心の角速度 [rad/s]
 *     dt        : 刻み時間 [s]
 *     taylor_th : sinc のテイラー展開閾値 [rad]
 *     dp_x      : 車体座標系での x 方向増分の格納先 [m]
 *     dp_y      : 車体座標系での y 方向増分の格納先 [m]
 * 出力:
 *     なし（dp_x, dp_y に書き込む）
 */
static void arc_increment(double v, double omega, double dt, double taylor_th,
                          double *dp_x, double *dp_y)
{
    const double half  = 0.5 * omega * dt;                  /* 回転増分の半分 */
    const double chord = v * dt * sinc(half, taylor_th);    /* 弧長を弦長へ換算 */

    /* 弦の方向は進行方向から半回転分だけ傾く */
    *dp_x = chord * cos(half);
    *dp_y = chord * sin(half);
}

/*
 * 一時刻前の LiDAR 姿勢とオドメトリから NDT の初期値を予測する
 *
 * 入力:
 *     prev      : 一時刻前の LiDAR 姿勢（世界座標系）
 *     v         : 車両中心の並進速度 [m/s]
 *     omega     : 車両中心の角速度 [rad/s]
 *     ext       : 車両中心から見た LiDAR の外部パラメータ
 *     dt        : 刻み時間 [s]
 *     taylor_th : sinc のテイラー展開閾値 [rad]
 * 出力:
 *     予測された LiDAR 姿勢（世界座標系）
 */
Pose6 predict_lidar_pose(const Pose6 *prev, double v, double omega,
                         const Pose6 *ext, double dt, double taylor_th)
{
    Pose6 next;

    /* LiDAR の yaw から車両中心の yaw を復元する */
    const double th_prev = prev->yaw - ext->yaw;
    const double d_th    = omega * dt;
    const double th_next = th_prev + d_th;

    /* 車体座標系での並進増分を求める */
    double dp_x, dp_y;
    arc_increment(v, omega, dt, taylor_th, &dp_x, &dp_y);

    /* 車両中心の並進を世界座標系へ回転する */
    const double trans_x = dp_x * cos(th_prev) - dp_y * sin(th_prev);
    const double trans_y = dp_x * sin(th_prev) + dp_y * cos(th_prev);

    /* 旋回によるオフセット点の振り回し（レバーアーム）分 */
    const double lever_x = ext->x * (cos(th_next) - cos(th_prev))
                         - ext->y * (sin(th_next) - sin(th_prev));
    const double lever_y = ext->x * (sin(th_next) - sin(th_prev))
                         + ext->y * (cos(th_next) - cos(th_prev));

    next.x = prev->x + trans_x + lever_x;
    next.y = prev->y + trans_y + lever_y;

    /* 水平面移動なので z, roll, pitch は不変、yaw のみ増分を加算する */
    next.z     = prev->z;
    next.roll  = prev->roll;
    next.pitch = prev->pitch;
    next.yaw   = normalize_angle(prev->yaw + d_th);

    return next;
}

/*
 * 位置姿勢を 4x4 同次変換行列へ変換する（NDT へ初期値として渡す形式）
 *
 * 入力:
 *     pose : 変換元の位置姿勢
 *     mat  : 4x4 行列の格納先（行優先）
 * 出力:
 *     なし（mat に書き込む）
 */
void pose_to_matrix(const Pose6 *pose, double mat[4][4])
{
    const double cr = cos(pose->roll),  sr = sin(pose->roll);
    const double cp = cos(pose->pitch), sp = sin(pose->pitch);
    const double cy = cos(pose->yaw),   sy = sin(pose->yaw);

    /* R = Rz(yaw) * Ry(pitch) * Rx(roll) */
    mat[0][0] = cy * cp;
    mat[0][1] = cy * sp * sr - sy * cr;
    mat[0][2] = cy * sp * cr + sy * sr;
    mat[1][0] = sy * cp;
    mat[1][1] = sy * sp * sr + cy * cr;
    mat[1][2] = sy * sp * cr - cy * sr;
    mat[2][0] = -sp;
    mat[2][1] = cp * sr;
    mat[2][2] = cp * cr;

    /* 並進成分 */
    mat[0][3] = pose->x;
    mat[1][3] = pose->y;
    mat[2][3] = pose->z;

    /* 最下行 */
    mat[3][0] = 0.0;
    mat[3][1] = 0.0;
    mat[3][2] = 0.0;
    mat[3][3] = 1.0;
}

/* ===== 使用例 ===== */

int main(void)
{
    /* 外部パラメータをパラメータ定義から組み立てる */
    const Pose6 ext = {X_OFFSET, Y_OFFSET, Z_OFFSET,
                       ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET};

    /* 一時刻前の LiDAR 姿勢（前回の NDT 収束結果を想定） */
    Pose6 pose = {10.0, 5.0, 1.20, 0.0, 0.0, 0.30};

    /* オドメトリ入力 */
    const double v     = 1.0;   /* [m/s]   */
    const double omega = 0.5;   /* [rad/s] */

    for (int i = 0; i < 3; i++) {
        pose = predict_lidar_pose(&pose, v, omega, &ext, DT, SINC_TAYLOR_TH);
        printf("step %d: x=%8.4f  y=%8.4f  z=%6.3f  yaw=%8.5f\n",
               i + 1, pose.x, pose.y, pose.z, pose.yaw);
    }

    /* NDT へ渡す 4x4 行列へ変換 */
    double init_guess[4][4];
    pose_to_matrix(&pose, init_guess);

    printf("\ninitial guess matrix:\n");
    for (int r = 0; r < 4; r++) {
        printf("  %8.4f %8.4f %8.4f %8.4f\n",
               init_guess[r][0], init_guess[r][1],
               init_guess[r][2], init_guess[r][3]);
    }

    return 0;
}
