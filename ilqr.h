#ifndef _ILQR_H
#define _ILQR_H

#include <ArduinoEigenDense.h>
#include <vector>
#include <cmath>

template <int NX, int NU>
class ILQRController
{
public:
    using State = Eigen::Matrix<float, NX, 1>;
    using Control = Eigen::Matrix<float, NU, 1>;
    using StateMat = Eigen::Matrix<float, NX, NX>;
    using ControlMat = Eigen::Matrix<float, NU, NU>;
    using StateControlMat = Eigen::Matrix<float, NX, NU>;
    using ControlStateMat = Eigen::Matrix<float, NU, NX>;

    struct Dynamics
    {
        virtual State f(const State &x, const Control &u) const = 0;
        virtual StateMat A(const State &x, const Control &u) const = 0;
        virtual StateControlMat B(const State &x, const Control &u) const = 0;
    };

    struct Cost
    {
        virtual float l(const State &x, const Control &u, int t) const = 0;
        virtual State lx(const State &x, const Control &u, int t) const = 0;
        virtual Control lu(const State &x, const Control &u, int t) const = 0;
        virtual StateMat lxx(const State &x, const Control &u, int t) const = 0;
        virtual ControlMat luu(const State &x, const Control &u, int t) const = 0;
        virtual ControlStateMat lux(const State &x, const Control &u, int t) const = 0;
    };

    ILQRController(int T, const Dynamics &dynamics, const Cost &cost, const bool verbose = false)
        : T_(T), dynamics_(dynamics), cost_(cost), verbose_(verbose)
    {
        xs_.resize(T_ + 1, State::Zero());
        us_.resize(T_, Control::Zero());
        Ks_.resize(T_, ControlStateMat::Zero());
        ks_.resize(T_, Control::Zero());

        u_min_ = Control::Constant(-0.5f); // 控制下限
        u_max_ = Control::Constant(0.5f);  // 控制上限
    }

    void solve(const State &x0, float atol = 1e-3, int max_iters = 50)
    {
        // 初始 rollout
        xs_[0] = x0;
        for (int t = 0; t < T_; t++)
            xs_[t + 1] = dynamics_.f(xs_[t], us_[t]);

        for (int iter = 0; iter < max_iters; iter++)
        {
            float dJ = backward_pass(); // 计算 ks_ / Ks_ 并返回 ΔJ
            forward_pass(x0);           // forward pass + 投影

            // 收敛判据
            float max_d = 0.0f;
            for (int t = 0; t < T_; t++)
                max_d = std::max(max_d, ks_[t].norm());
            if (verbose_)
            {
                Serial.print("iter=");
                Serial.print(iter);
                Serial.print("  ΔJ=");
                Serial.print(dJ);
                Serial.print("  max|d|=");
                Serial.println(max_d);

                if (std::abs(dJ) < atol)
                {
                    Serial.print("iLQR converged at iteration ");
                    Serial.println(iter);
                    break;
                }
            }
        }
    }

    const std::vector<State> &getStates() const { return xs_; }
    const std::vector<Control> &getControls() const { return us_; }
    const std::vector<ControlStateMat> &getFeedbackGains() const { return Ks_; }
    const std::vector<Control> &getFeedforward() const { return ks_; }

    void setControlLimits(const Control &umin, const Control &umax)
    {
        u_min_ = umin;
        u_max_ = umax;
    }

private:
    float backward_pass()
    {
        State Vx = State::Zero();
        StateMat Vxx = StateMat::Zero();
        float dJ = 0.0f;

        constexpr float max_ks = 0.3f; // 信赖域大小，可根据控制尺度调整
        constexpr float reg = 1e-4f;   // Quu 正则化

        for (int t = T_ - 1; t >= 0; t--)
        {
            const auto &x = xs_[t];
            const auto &u = us_[t];

            StateMat A = dynamics_.A(x, u);
            StateControlMat B = dynamics_.B(x, u);

            State lx = cost_.lx(x, u, t);
            Control lu = cost_.lu(x, u, t);
            StateMat lxx = cost_.lxx(x, u, t);
            ControlMat luu = cost_.luu(x, u, t);
            ControlStateMat lux = cost_.lux(x, u, t);

            State Qx = lx + A.transpose() * Vx;
            Control Qu = lu + B.transpose() * Vx;
            StateMat Qxx = lxx + A.transpose() * Vxx * A;
            ControlMat Quu = luu + B.transpose() * Vxx * B;
            ControlMat Quu_reg = Quu + reg * ControlMat::Identity(); // Quu 正则化
            ControlStateMat Qux = lux + B.transpose() * Vxx * A;

            // 无约束求解
            Control ks_unconstr = -Quu_reg.ldlt().solve(Qu);
            ControlStateMat Ks_unconstr = -Quu_reg.ldlt().solve(Qux);

            // 信赖域限制
            float norm_ks = ks_unconstr.norm();
            if (norm_ks > max_ks)
                ks_unconstr *= max_ks / norm_ks;

            // 投影到控制上下限
            /*
            Control u_proj = u + ks_unconstr;
            for (int i = 0; i < NU; i++)
            {
                if (u_proj(i) < -0.5f)
                    ks_unconstr(i) = -0.5f - u(i);
                if (u_proj(i) > 0.5f)
                    ks_unconstr(i) = 0.5f - u(i);
            }*/

            ks_[t] = ks_unconstr;
            Ks_[t] = Ks_unconstr;

            Vx = Qx + Ks_[t].transpose() * Quu * ks_[t] + Ks_[t].transpose() * Qu + Qux.transpose() * ks_[t];
            Vxx = Qxx + Ks_[t].transpose() * Quu * Ks_[t] + Ks_[t].transpose() * Qux + Qux.transpose() * Ks_[t];

            dJ += Qu.dot(ks_[t]);
        }

        return dJ;
    }

    void forward_pass(const State &x0)
    {
        xs_[0] = x0;
        for (int t = 0; t < T_; t++)
        {
            // feedforward + feedback
            Control u_pred = us_[t] + ks_[t];
            // 投影到控制范围
            for (int i = 0; i < NU; i++)
                u_pred(i) = std::min(std::max(u_pred(i), u_min_(i)), u_max_(i));

            us_[t] = u_pred;
            xs_[t + 1] = dynamics_.f(xs_[t], us_[t]);
        }
    }

private:
    int T_;
    const Dynamics &dynamics_;
    const Cost &cost_;
    bool verbose_; // for debugging

    std::vector<State> xs_;
    std::vector<Control> us_;
    std::vector<ControlStateMat> Ks_;
    std::vector<Control> ks_;

    Control u_min_;
    Control u_max_;
};

#endif // _ILQR_H
