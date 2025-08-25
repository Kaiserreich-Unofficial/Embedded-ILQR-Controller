#ifndef _DYNAMICS_H
#define _DYNAMICS_H

#include "ilqr.h"

template <int NX, int NU>
struct USVDynamics : public ILQRController<NX, NU>::Dynamics
{
    using State = typename ILQRController<NX, NU>::State;
    using Control = typename ILQRController<NX, NU>::Control;
    using StateMat = typename ILQRController<NX, NU>::StateMat;
    using ControlMat = typename ILQRController<NX, NU>::ControlMat;
    using StateControlMat = typename ILQRController<NX, NU>::StateControlMat;
    using ControlStateMat = typename ILQRController<NX, NU>::ControlStateMat;

    float dt;
    float eps = 1e-5f;

    USVDynamics(float dt_) : dt(dt_) {}

    // 连续动力学
    typename ILQRController<NX, NU>::State f_continuous(
        const typename ILQRController<NX, NU>::State &x,
        const typename ILQRController<NX, NU>::Control &u) const
    {
        using State = typename ILQRController<NX, NU>::State;
        State dx;
        float psi = x(2);
        float u_ship = x(3);
        float v = x(4);
        float r = x(5);
        float Tl = u(0);
        float Tr = u(1);

        dx(0) = u_ship * std::cos(psi) - v * std::sin(psi); // x_dot
        dx(1) = u_ship * std::sin(psi) + v * std::cos(psi); // y_dot
        dx(2) = r;                                          // psi_dot
        dx(3) = -0.1189f * u_ship - 0.1131f * u_ship * std::fabs(u_ship) + 0.72696f * v * r + 1.04965f * Tl + 0.72806f * Tr - 0.05309f * r * r;
        dx(4) = 0.1577f * v + 0.0365f * r - 0.7457f * v * std::fabs(v) - 0.0348f * r * std::fabs(r) - 0.22145f * u_ship * v - 0.0678f * u_ship * r - 0.0899f * Tl + 0.1015f * Tr;
        dx(5) = 1.1100f * v - 0.2210f * r - 2.7120f * v * std::fabs(v) - 0.2083f * r * std::fabs(r) - 0.6508f * u_ship * v - 0.6683f * u_ship * r - 1.9953f * Tl + 2.1045f * Tr;

        return dx;
    }

    // RK4 积分
    typename ILQRController<NX, NU>::State rk4_step(
        const typename ILQRController<NX, NU>::State &x,
        const typename ILQRController<NX, NU>::Control &u) const
    {
        using State = typename ILQRController<NX, NU>::State;
        State k1 = f_continuous(x, u);
        State k2 = f_continuous(x + 0.5f * dt * k1, u);
        State k3 = f_continuous(x + 0.5f * dt * k2, u);
        State k4 = f_continuous(x + dt * k3, u);
        return x + dt / 6.0f * (k1 + 2 * k2 + 2 * k3 + k4);
    }

    // 重写 ILQRController::Dynamics 接口
    typename ILQRController<NX, NU>::State f(
        const typename ILQRController<NX, NU>::State &x,
        const typename ILQRController<NX, NU>::Control &u) const override
    {
        return rk4_step(x, u);
    }

    typename ILQRController<NX, NU>::StateMat A(
        const typename ILQRController<NX, NU>::State &x,
        const typename ILQRController<NX, NU>::Control &u) const override
    {
        using State = typename ILQRController<NX, NU>::State;
        using StateMat = typename ILQRController<NX, NU>::StateMat;
        StateMat A_d;
        State x_next = rk4_step(x, u);

        for (int i = 0; i < NX; ++i)
        {
            State x_pert = x;
            x_pert(i) += eps;
            State x_next_pert = rk4_step(x_pert, u);
            A_d.col(i) = (x_next_pert - x_next) / eps;
        }
        return A_d;
    }

    typename ILQRController<NX, NU>::StateControlMat B(
        const typename ILQRController<NX, NU>::State &x,
        const typename ILQRController<NX, NU>::Control &u) const override
    {
        using State = typename ILQRController<NX, NU>::State;
        using StateControlMat = typename ILQRController<NX, NU>::StateControlMat;
        StateControlMat B_d;
        State x_next = rk4_step(x, u);

        for (int i = 0; i < NU; ++i)
        {
            typename ILQRController<NX, NU>::Control u_pert = u;
            u_pert(i) += eps;
            State x_next_pert = rk4_step(x, u_pert);
            B_d.col(i) = (x_next_pert - x_next) / eps;
        }
        return B_d;
    }
};

#endif // _DYNAMICS_H
