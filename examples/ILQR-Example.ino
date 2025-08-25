#include <Arduino.h>
#include "ilqr.h"
#include "dynamics.h"
#include "cost.h"
#include <vector>

constexpr int NX = 6;
constexpr int NU = 2;
constexpr int T = 35;
constexpr float dt = 0.1f;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // 等待串口打开
    // -----------------------------
    // 1. 创建动力学对象
    // -----------------------------
    USVDynamics<NX, NU> dyn(dt);

    // -----------------------------
    // 2. 创建 QuadraticCost 对象
    // -----------------------------
    QuadraticCost<NX, NU> cost(T);

    float r = 6.0f;
    float omega = 0.1f;
    float u_ref = r * omega;

    // 设置圆轨迹参考
    for (int t = 0; t <= T; t++)
    {
        ILQRController<NX, NU>::State xr;
        xr.setZero();
        float time = t * dt;
        xr(0) = r * cos(omega * time); // x
        xr(1) = r * sin(omega * time); // y
        xr(2) = omega * time;          // psi
        xr(3) = u_ref;                 // surge (参考速度)
        xr(4) = 0;                     // sway
        xr(5) = omega;                 // yaw rate
        cost.setRef(t, xr);
    }

    // -----------------------------
    // 3. 创建 ILQRController 对象
    // -----------------------------
    ILQRController<NX, NU> ilqr(T, dyn, cost);

    // 初始状态
    ILQRController<NX, NU>::State x0;
    x0.setZero();
    x0(0) = r;
    x0(1) = 0;
    x0(2) = 0;
    x0(3) = u_ref;
    x0(4) = 0;
    x0(5) = omega;

    // -----------------------------
    // 4. 求解 iLQR
    // -----------------------------
    unsigned long t_start = millis();
    ilqr.solve(x0);
    unsigned long t_end = millis();

    Serial.print("iLQR solve time (ms): ");
    Serial.println(t_end - t_start);

    // -----------------------------
    // 5. 获取预测轨迹和控制输入
    // -----------------------------
    const auto &X_pred = ilqr.getStates();
    const auto &U_pred = ilqr.getControls();

    Serial.println("Predicted states (x, y, psi, u, v, r):");
    for (int t = 0; t <= T; t++)
    {
        Serial.print("t=");
        Serial.print(t * dt, 2);
        Serial.print("  ");
        for (int i = 0; i < NX; i++)
        {
            Serial.print(X_pred[t](i), 3);
            Serial.print(" ");
        }
        Serial.println();
    }

    Serial.println("Predicted controls (Tl, Tr):");
    for (int t = 0; t < T; t++)
    {
        Serial.print("t=");
        Serial.print(t * dt, 2);
        Serial.print("  ");
        for (int i = 0; i < NU; i++)
        {
            Serial.print(U_pred[t](i), 3);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void loop()
{
    // 这里可以实现循环控制，每 dt 更新一次
}
