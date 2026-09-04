#pragma once

#include "cartesian_impedance_controller.hpp"

namespace project_config
{

struct ControllerConfig
{
    using Controller = impedance::CartesianImpedanceController;
    using Vec3 = Eigen::Vector3d;
    using Vec6 = Controller::Vec6;
    using Vec7 = Controller::Vec7;

    // ============================================================
    // CONTROL LOOP
    // ============================================================

    double update_hz = 1000.0;

    // Terminal dashboard refresh period.
    double dashboard_period_s = 0.20;

    // <= 0.0 => run until Ctrl+C.
    double experiment_duration_s = 30.0;



    // CARTESIAN REFERENCE OFFSET
    //
    // Desired Cartesian pose relative to the pose at startup.
    //
    // Position:
    //   [dx, dy, dz]
    //   expressed in ROBOT BASE frame.
    //   Units: meters.
    //
    // Orientation:
    //   [roll, pitch, yaw]
    //   relative to the INITIAL end-effector orientation.
    //   Units: degrees.
    //
    // ============================================================

    bool use_cartesian_offset = true;


    // Position offset [m]
    Eigen::Vector3d position_offset =
        (Eigen::Vector3d()
            << 0.05,   // +5 cm in base X
            0.00,   // Y
            0.00)   // Z
        .finished();


    // Orientation offset [deg]
    Eigen::Vector3d orientation_offset_rpy_deg =
        (Eigen::Vector3d()
            << 0.0,    // Roll
            0.0,    // Pitch
            0.0)    // Yaw
        .finished();


    // ============================================================
    // CARTESIAN STIFFNESS
    // ============================================================
    //
    // [Kx, Ky, Kz, Krx, Kry, Krz]
    //
    // Translation: N/m
    // Rotation:    Nm/rad
    //
    // ============================================================

    Vec6 stiffness =
        (Vec6()
            << 100.0, 100.0, 100.0,
               10.0,  10.0,  10.0)
        .finished();


    // ============================================================
    // CARTESIAN DAMPING FACTORS
    // ============================================================
    //
    // Controller rule:
    //
    //     D_i = damping_factor_i * 2*sqrt(K_i)
    //
    // ============================================================

    Vec6 damping_factors =
        Vec6::Ones();


    // ============================================================
    // NULLSPACE
    // ============================================================
    //
    // Completely safe to enable in SHADOW mode because
    // ENABLE_IMPEDANCE_HARDWARE remains false in main.
    //
    // q_null_des = q_initial + nullspace_offset
    //
    // Offset units: rad.
    //
    // Example 5 deg on joint 3:
    //
    // nullspace_offset =
    //     (Vec7() << 0, 0, 0.0872665, 0, 0, 0, 0).finished();
    //
    // ============================================================

    bool enable_nullspace = false;

    double nullspace_stiffness = 5.0;
    double nullspace_damping_factor = 1.0;

    Vec7 nullspace_offset =
        Vec7::Zero();


    // ============================================================
    // DESIRED CARTESIAN WRENCH
    // ============================================================
    //
    // [Fx, Fy, Fz, Mx, My, Mz]
    //
    // Force:  N
    // Moment: Nm
    //
    // Example:
    //
    // wrench =
    //     (Vec6() << 0, 0, -5, 0, 0, 0).finished();
    //
    // ============================================================

    bool enable_wrench = false;

    Vec6 wrench =
        Vec6::Zero();




    // ============================================================
    // FILTERS
    // ============================================================
    //
    // IMPORTANT:
    // This matches YOUR custom controller API:
    //
    // setFiltering(
    //     update_hz,
    //     pose_filter,
    //     stiffness_filter,
    //     wrench_filter,
    //     nullspace_filter
    // );
    //
    // Each filter must be in (0,1].
    //
    // ============================================================

    double pose_filter = 0.10;
    double stiffness_filter = 0.10;
    double wrench_filter = 0.10;
    double nullspace_filter = 0.10;


    // ============================================================
    // IMPEDANCE TORQUE SLEW-RATE LIMIT
    // ============================================================

    double delta_tau_max = 1.0;


    // ============================================================
    // GRAVITY TRANSITIONS
    // ============================================================

    int gravity_warmup_cycles = 200;
    int gravity_shutdown_cycles = 200;


    // ============================================================
    // FUTURE HARDWARE BLEND
    // ============================================================
    //
    // Used ONLY when compile-time hardware impedance is enabled.
    //
    // tau_hw = g(q) + alpha*tau_imp
    //
    // alpha grows 0 -> 1 in this time.
    //
    // ============================================================

    double impedance_blend_time_s = 2.0;


    // ============================================================
    // SHADOW DIAGNOSTIC THRESHOLDS
    // ============================================================
    //
    // These produce WARNINGS only.
    // NaN/Inf and REAL hardware current violations produce a stop.
    //
    // ============================================================

    double max_shadow_position_error_m = 0.25;
    double max_shadow_rotation_error_rad = 1.0;

    double max_shadow_linear_speed_m_s = 1.0;
    double max_shadow_angular_speed_rad_s = 3.0;

    double max_shadow_impedance_torque_nm = 30.0;
};

} // namespace project_config
