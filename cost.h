#ifndef _COST_H_
#define _COST_H_

#include "ilqr.h"
#include <vector>

template <int NX, int NU>
struct QuadraticCost : public ILQRController<NX, NU>::Cost
{
    using State = typename ILQRController<NX, NU>::State;
    using Control = typename ILQRController<NX, NU>::Control;
    using StateMat = typename ILQRController<NX, NU>::StateMat;
    using ControlMat = typename ILQRController<NX, NU>::ControlMat;
    using StateControlMat = typename ILQRController<NX, NU>::StateControlMat;
    using ControlStateMat = typename ILQRController<NX, NU>::ControlStateMat;

    Eigen::Matrix<float, NX, NX> Q;
    Eigen::Matrix<float, NU, NU> R;
    std::vector<State> x_ref; // 参考轨迹

    QuadraticCost(int T)
    {
        Q = 10 * Eigen::Matrix<float, NX, NX>::Identity();
        R = 0.1 * Eigen::Matrix<float, NU, NU>::Identity();
        x_ref.resize(T + 1, State::Zero());
    }

    void setRef(int t, const State &xr)
    {
        if (t >= 0 && t < x_ref.size())
            x_ref[t] = xr;
    }

    const State &getRef(int t) const
    {
        if (t >= 0 && t < x_ref.size())
            return x_ref[t];
        return x_ref.back();
    }

    // 增加时间索引 t 参数
    float l(const State &x, const Control &u, int t) const
    {
        State dx = x - getRef(t);
        return 0.5f * (dx.transpose() * Q * dx + u.transpose() * R * u)(0, 0);
    }

    State lx(const State &x, const Control &u, int t) const
    {
        return Q * (x - getRef(t));
    }

    Control lu(const State &x, const Control &u, int /*t*/) const
    {
        return R * u;
    }

    StateMat lxx(const State & /*x*/, const Control & /*u*/, int /*t*/) const
    {
        return Q;
    }

    ControlMat luu(const State & /*x*/, const Control & /*u*/, int /*t*/) const
    {
        return R;
    }

    ControlStateMat lux(const State & /*x*/, const Control & /*u*/, int /*t*/) const
    {
        return ControlStateMat::Zero();
    }
};

#endif // _COST_H_
