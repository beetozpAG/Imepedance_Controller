#pragma once

#include <Eigen/Dense>

namespace impedance
{

class CartesianImpedanceController
{
public:
    using Vec7 = Eigen::Matrix<double, 7, 1>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Mat67 = Eigen::Matrix<double, 6, 7>;
    using Mat66 = Eigen::Matrix<double, 6, 6>;

    struct Output
    {
        Vec7 tau_task = Vec7::Zero();
        Vec7 tau_null = Vec7::Zero();
        Vec7 tau_ext = Vec7::Zero();
        Vec7 tau_impedance = Vec7::Zero();

        Vec6 pose_error = Vec6::Zero();
    };

    CartesianImpedanceController();

    void initialize(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Vec7& q);

    void setReferencePose(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation);

    void setNullspaceReference(
        const Vec7& q_null);

    void setCartesianStiffness(
        const Vec6& stiffness);

    void setNullspaceStiffness(
        double stiffness);

    void setDampingFactors(
        const Vec6& cartesian_factors,
        double nullspace_factor);

    void setWrench(
        const Vec6& wrench);

    void setFiltering(
        double update_hz,
        double pose_filter,
        double stiffness_filter,
        double wrench_filter,
        double nullspace_filter);

    void setMaxTorqueDelta(
        double delta_tau_max);

    Output compute(
        const Vec7& q,
        const Vec7& dq,
        const Eigen::Vector3d& position,
        Eigen::Quaterniond orientation,
        const Mat67& J);

private:
    static Eigen::Vector3d orientationError(
        const Eigen::Quaterniond& desired,
        Eigen::Quaterniond current);

    static double dampingRule(
        double stiffness);

    static double filterStep(
        double update_hz,
        double filter);

    static Eigen::MatrixXd pseudoInverse(
        const Eigen::MatrixXd& matrix);

    void updateFilteredTargets();

private:
    Mat66 K_ = Mat66::Zero();
    Mat66 K_target_ = Mat66::Zero();

    Mat66 D_ = Mat66::Zero();
    Mat66 D_target_ = Mat66::Zero();

    Vec6 damping_factors_ =
        Vec6::Ones();

    double K_null_ = 0.0;
    double K_null_target_ = 0.0;

    double D_null_ = 0.0;
    double D_null_target_ = 0.0;

    double damping_null_factor_ = 1.0;

    Eigen::Vector3d p_d_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d p_d_target_ =
        Eigen::Vector3d::Zero();

    Eigen::Quaterniond R_d_ =
        Eigen::Quaterniond::Identity();

    Eigen::Quaterniond R_d_target_ =
        Eigen::Quaterniond::Identity();

    Vec7 q_null_ =
        Vec7::Zero();

    Vec7 q_null_target_ =
        Vec7::Zero();

    Vec6 wrench_ =
        Vec6::Zero();

    Vec6 wrench_target_ =
        Vec6::Zero();

    Vec7 tau_last_ =
        Vec7::Zero();

    double update_hz_ = 1000.0;

    double pose_filter_ = 0.1;
    double stiffness_filter_ = 0.1;
    double wrench_filter_ = 0.1;
    double nullspace_filter_ = 0.1;

    double delta_tau_max_ = 1.0;
};

}  // namespace impedance
