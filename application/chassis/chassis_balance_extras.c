#include "chassis_balance_extras.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "math.h"

/**
 * @brief 获取K矩阵
 * @param[in]  l 腿长
 * @param[out] k K矩阵
 */
void GetK(float l, float k[2][6], bool is_take_off)
{
    float t1 = l;
    float t2 = l * l;
    float t3 = l * l * l;
    k[0][0] = -157.0203f * t3 + 179.8485f * t2 - 85.4172f * t1 + 0.0537f;
    k[0][1] = -1.0899f * t3 + 3.0072f * t2 - 6.6735f * t1 + 0.1905f;
    k[0][2] = -27.7037f * t3 + 27.7682f * t2 - 9.7108f * t1 - 0.2497f;
    k[0][3] = -32.3706f * t3 + 32.7508f * t2 - 12.0579f * t1 - 0.3533f;
    k[0][4] = -38.5250f * t3 + 54.5544f * t2 - 30.4327f * t1 + 8.2300f;
    k[0][5] = -5.2760f * t3 + 8.9537f * t2 - 5.7839f * t1 + 1.8671f;
    k[1][0] = 152.7053f * t3 - 130.2649f * t2 + 28.7526f * t1 + 5.6772f;
    k[1][1] = 16.2673f * t3 - 16.3037f * t2 + 5.2455f * t1 + 0.1397f;
    k[1][2] = -21.5602f * t3 + 27.7447f * t2 - 13.9335f * t1 + 3.1997f;
    k[1][3] = -24.4104f * t3 + 31.6150f * t2 - 16.0527f * t1 + 3.7713f;
    k[1][4] = 224.1583f * t3 - 227.6099f * t2 + 81.5551f * t1 + 0.1116f;
    k[1][5] = 50.1510f * t3 - 51.5745f * t2 + 18.8992f * t1 - 0.3485f;

    if (is_take_off) {
        k[0][0] = 0;
        k[0][1] = 0;
        k[0][2] = 0;
        k[0][3] = 0;
        k[0][4] = 0;
        k[0][5] = 0;
        // k[1][0] = 0;
        // k[1][1] = 0;
        k[1][2] = 0;
        k[1][3] = 0;
        k[1][4] = 0;
        k[1][5] = 0;
    }
}

/**
 * @brief 通过关节phi1和phi4的值获取L0和Phi0
 * @param[in]  phi1
 * @param[in]  phi4
 * @param[out] L0_Phi0 L0和Phi0
 */
void GetL0AndPhi0(float phi1, float phi4, float L0_Phi0[2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, phi2, XC, YC;
    float L0, Phi0;
    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    Phi0 = atan2(YC, (XC - LEG_L5 / 2));

    L0_Phi0[0] = L0;
    L0_Phi0[1] = Phi0;
}

/**
 * @brief 获取dL0和dPhi0
 * @param[in]  J 雅可比矩阵
 * @param[in]  d_phi1 
 * @param[in]  d_phi4 
 */
void GetdL0AnddPhi0(float J[2][2], float d_phi1, float d_phi4, float dL0_dPhi0[2])
{
    // clang-format off
    float d_l0   = J[0][0] * d_phi1 + J[0][1] * d_phi4;
    float d_phi0 = J[1][0] * d_phi1 + J[1][1] * d_phi4;
    // clang-format on
    dL0_dPhi0[0] = d_l0;
    dL0_dPhi0[1] = d_phi0;
}

/**
 * @brief 获取腿部摆杆的等效力
 * @param[in]  J 雅可比矩阵
 * @param[in]  T1 
 * @param[in]  T2 
 * @param[out] F 0-F0 1-Tp
 */
void GetLegForce(float J[2][2], float T1, float T2, float F[2])
{
    float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    // clang-format off
    float inv_J[4] = {J[1][1] / det, -J[0][1] / det, 
                     -J[1][0] / det,  J[1][1] / det};
    // clang-format on
    //F = (inv_J.') * T
    float F0 = inv_J[0] * T1 + inv_J[2] * T2;
    float Tp = inv_J[1] * T1 + inv_J[3] * T2;

    F[0] = F0;
    F[1] = Tp;
}

/**
 * @brief 计算雅可比矩阵
 * @param phi1 
 * @param phi4 
 * @param J 
 */
void CalcJacobian(float phi1, float phi4, float J[2][2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, XC, YC;
    float phi2, phi3;
    float L0, phi0;
    float j11, j12, j21, j22;

    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    phi3 = atan2(YB - YD + LEG_L2 * sin(phi2), XB - XD + LEG_L2 * cos(phi2));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    phi0 = atan2(YC, XC - LEG_L5 / 2);

    j11 = (LEG_L1 * sin(phi0 - phi3) * sin(phi1 - phi2)) / sin(phi3 - phi2);
    j12 = (LEG_L4 * sin(phi0 - phi2) * sin(phi3 - phi4)) / sin(phi3 - phi2);
    j21 = (LEG_L1 * cos(phi0 - phi3) * sin(phi1 - phi2)) / (L0 * sin(phi3 - phi2));
    j22 = (LEG_L4 * cos(phi0 - phi2) * sin(phi3 - phi4)) / (L0 * sin(phi3 - phi2));

    J[0][0] = j11;
    J[0][1] = j12;
    J[1][0] = j21;
    J[1][1] = j22;
}

/**
 * @brief 计算VMC
 * @param[in]  F0 沿杆方向的力
 * @param[in]  Tp 髋关节力矩
 * @param[in]  J 雅可比矩阵
 * @param[out] T 2个关节的输出力矩
 */
void CalcVmc(float F0, float Tp, float J[2][2], float T[2])
{
    // clang-format off
    float JT[2][2] = {{J[0][0],J[1][0]}, // 转置矩阵
                      {J[0][1],J[1][1]}};
    float F[2] = {F0, Tp};
    // clang-format on
    float T1 = JT[0][0] * F[0] + JT[0][1] * F[1];
    float T2 = JT[1][0] * F[0] + JT[1][1] * F[1];

    T[0] = T1;
    T[1] = T2;
}

/**
 * @brief 通过L0和Phi0的值计算关节phi1和phi4
 * @param[in]  phi0
 * @param[in]  l0
 * @param[out] phi1_phi4 phi1和phi4
 * @note 用于位置控制时求逆解
 */
void CalcPhi1AndPhi4(float phi0, float l0, float phi1_phi4[2])
{
    float L5_2_pow;
    float Lca2, Lce2;
    float cos_phi11, cos_phi12, cos_phi41, cos_phi42;
    float phi11, phi12, phi41, phi42;
    float phi1, phi4;

    L5_2_pow = (LEG_L5 / 2) * (LEG_L5 / 2);  //(LEG_L5 / 2)^2
    Lca2 = l0 * l0 + L5_2_pow + l0 * LEG_L5 * cos(phi0);
    Lce2 = l0 * l0 + L5_2_pow - l0 * LEG_L5 * cos(phi0);

    cos_phi11 = (L5_2_pow + Lca2 - l0 * l0) / (LEG_L5 * sqrt(Lca2));
    cos_phi12 = (LEG_L1 * LEG_L1 + Lca2 - LEG_L2 * LEG_L2) / (2 * LEG_L1 * sqrt(Lca2));
    cos_phi41 = (L5_2_pow + Lce2 - l0 * l0) / (LEG_L5 * sqrt(Lce2));
    cos_phi42 = (LEG_L4 * LEG_L4 + Lce2 - LEG_L3 * LEG_L3) / (2 * LEG_L5 * sqrt(Lce2));

    phi11 = acos(cos_phi11);
    phi12 = acos(cos_phi12);

    phi41 = acos(cos_phi41);
    phi42 = acos(cos_phi42);

    phi1 = phi11 + phi12;
    phi4 = M_PI - (phi41 + phi42);

    phi1_phi4[0] = phi1;
    phi1_phi4[1] = phi4;
}

/**
 * @brief 通过当前底盘姿态和目标roll角计算两腿长度期望差值
 * @param[in]  Ld0 (m)当前左右腿长度差值(L0l - L0r)
 * @param[in]  theta0 (rad)当前底盘roll角
 * @param[in]  theta1 (rad)目标roll角
 * @return 两腿长度期望差值(m)(L1l - L1r)
 */
float CalcLegLengthDiff(float Ld0, float theta0, float theta1)
{
    return WHEEL_BASE * tanf(theta1) -
           cosf(theta0) / cosf(theta1) * (WHEEL_BASE * tanf(theta0) - Ld0);
}

/**
 * @brief 双腿腿长协调控制，维持腿长目标在范围内，同时尽可能达到两腿目标差值
 * @param[in]  Ll_ref   (m)左腿长度指针
 * @param[in]  Lr_ref   (m)右腿长度指针
 * @param[in]  diff (m)腿长差值
 * @param[in]  add  (m)腿长差值补偿量
 */
void CoordinateLegLength(float * Ll_ref, float * Lr_ref, float diff, float add)
{
    *Ll_ref = *Ll_ref + diff * 0.5f + add;
    *Lr_ref = *Lr_ref - diff * 0.5f - add;

    // float delta = *Ll_ref - *Lr_ref;
    // if (delta > MAX_LEG_LENGTH - MIN_LEG_LENGTH) {
    //     *Ll_ref = MAX_LEG_LENGTH;
    //     *Lr_ref = MIN_LEG_LENGTH;
    //     return;
    // } else if (delta < MIN_LEG_LENGTH - MAX_LEG_LENGTH) {
    //     *Ll_ref = MIN_LEG_LENGTH;
    //     *Lr_ref = MAX_LEG_LENGTH;
    //     return;
    // }

    //先判断短腿范围，再判断长腿范围
    float * short_leg = *Ll_ref < *Lr_ref ? Ll_ref : Lr_ref;
    float * long_leg = *Ll_ref < *Lr_ref ? Lr_ref : Ll_ref;
    float move = 0;
    move = MIN_LEG_LENGTH - *short_leg;
    if (move > 0) {
        *short_leg += move;
        *long_leg += move;
    }
    if (*long_leg > MAX_LEG_LENGTH) {
        *long_leg = MAX_LEG_LENGTH;
    }
}

#endif /* CHASSIS_BALANCE */
