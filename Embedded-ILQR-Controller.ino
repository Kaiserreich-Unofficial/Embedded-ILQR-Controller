#include <Arduino.h>
#include "ilqr.h"
#include "dynamics.h"
#include "cost.h"
#include <vector>

constexpr int NX = 6;
constexpr int NU = 2;
constexpr int T = 35;      // MPC预测步长
constexpr float dt = 0.1f; // 控制周期

USVDynamics<NX, NU> dyn(dt);
QuadraticCost<NX, NU> cost(T);
ILQRController<NX, NU> mpc(T, dyn, cost, false);

// 当前状态
ILQRController<NX, NU>::State x;

// 圆轨迹参数
float r = 10.0f;
float omega = 0.1f;
float u_ref = r * omega;

unsigned long last_update = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // 等待串口打开

    // 初始状态
    x.setZero();
    x(0) = r;
    x(1) = 0;
    x(2) = 0;
    x(3) = u_ref;
    x(4) = 0;
    x(5) = omega;

    Serial.println("MPC simulation started");
}

// 生成参考轨迹
void generateRefPath(int start_idx)
{
    for (int t = 0; t < T; t++)
    {
        ILQRController<NX, NU>::State xr;
        xr.setZero();
        float time = (start_idx + t) * dt;
        xr(0) = r * cos(omega * time); // x
        xr(1) = r * sin(omega * time); // y
        xr(2) = omega * time;          // psi
        xr(3) = u_ref;                 // surge
        xr(4) = 0;                     // sway
        xr(5) = omega;                 // yaw rate
        cost.setRef(t, xr);
    }
}

void loop()
{
    unsigned long now = millis();
    if (now - last_update >= dt * 1000)
    {
        last_update = now;

        // -----------------------------
        // 1. 生成当前 MPC 时间窗的参考轨迹
        // -----------------------------
        static int step_idx = 0;
        generateRefPath(step_idx);

        // -----------------------------
        // 2. 求解 iLQR/MPC，并计时
        // -----------------------------
        unsigned long t_start = millis();
        mpc.solve(x);
        unsigned long t_end = millis();
        unsigned long solve_time = t_end - t_start;

        // -----------------------------
        // 3. 获取当前控制量
        // -----------------------------
        const auto &U_pred = mpc.getControls();
        ILQRController<NX, NU>::Control u_cur = U_pred[0]; // 当前时刻控制

        // -----------------------------
        // 4. 更新状态（Euler积分）
        // -----------------------------
        x = dyn.f(x, u_cur);

        // -----------------------------
        // 5. 输出当前状态及求解时间
        // -----------------------------
        Serial.print("t=");
        Serial.print(step_idx * dt, 2);
        Serial.print("  x=");
        Serial.print(x(0), 3);
        Serial.print(" y=");
        Serial.print(x(1), 3);
        Serial.print(" psi=");
        Serial.print(x(2), 3);
        Serial.print(" u=");
        Serial.print(x(3), 3);
        Serial.print(" v=");
        Serial.print(x(4), 3);
        Serial.print(" r=");
        Serial.print(x(5), 3);
        Serial.print("  solve_time(ms)=");
        Serial.println(solve_time);

        step_idx++;
    }
}
