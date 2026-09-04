#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <boost/asio/io_context.hpp>

#include <data_logger.h>

#include "Matrix.h"
#include "Gen3_model.h"
#include "kinova_robot.h"
#include "udp_server.h"

#include "cartesian_impedance_controller.hpp"
#include "controller_config.hpp"


// ============================================================
// NAMESPACES / TYPES
// ============================================================

using namespace matrix;
using namespace KinovaGen3;

using kinova_robot::KinovaRobotLowLevel;
using impedance::CartesianImpedanceController;

using Vec7E = CartesianImpedanceController::Vec7;
using Vec6E = CartesianImpedanceController::Vec6;
using Mat67E = CartesianImpedanceController::Mat67;

using data_logger::DataLogger;
using data_logger::field;
using named_tuple::operator""_nm;




constexpr bool ENABLE_IMPEDANCE_HARDWARE = false;

constexpr const char* ROBOT_IP = "192.168.1.10";
constexpr const char* USERNAME = "admin";
constexpr const char* PASSWORD = "admin";

constexpr float PI = 3.14159265358979323846f;



const project_config::ControllerConfig cfg{};


template <typename T, size_t N, size_t M>
struct data_logger::csv::csv_traits<matrix::Matrix<T, N, M>>
    : csv_traits<std::array<std::array<T, M>, N>>
{
    template <size_t I>
        requires(I < N)
    static constexpr decltype(auto)
    get(const matrix::Matrix<T, N, M>& mat)
    {
        return std::get<I>(mat.data);
    }
};


// LOGGER


using ExperimentLogger =
    DataLogger<
        field<float,  "t">,

        field<vec<7>, "q">,
        field<vec<7>, "dq">,

        field<vec<7>, "pose">,
        field<vec<6>, "pose_err">,
        field<vec<6>, "cart_vel">,

        field<vec<7>, "tau_task">,
        field<vec<7>, "tau_null">,
        field<vec<7>, "tau_ext">,
        field<vec<7>, "tau_imp">,

        field<vec<7>, "tau_g">,
        field<vec<7>, "tau_virtual">,
        field<vec<7>, "tau_hw">,

        field<vec<7>, "current_virtual">,
        field<vec<7>, "current_cmd">,
        field<vec<7>, "current_meas">,
        field<vec<7>, "tau_meas">
    >;

ExperimentLogger logger;



volatile std::sig_atomic_t stop_requested = 0;

void signalHandler(int)
{
    stop_requested = 1;
}


Vec7E toEigen7(const vec<7>& v)
{
    Vec7E result;

    for (size_t i = 0; i < 7; ++i)
    {
        result(static_cast<Eigen::Index>(i)) =
            static_cast<double>(v[i][0]);
    }

    return result;
}


Mat67E toEigenJacobian(const mat<6, 7>& J)
{
    Mat67E result;

    for (size_t r = 0; r < 6; ++r)
    {
        for (size_t c = 0; c < 7; ++c)
        {
            result(
                static_cast<Eigen::Index>(r),
                static_cast<Eigen::Index>(c)
            ) =
                static_cast<double>(J[r][c]);
        }
    }

    return result;
}


vec<7> eigenToVec7(const Vec7E& v)
{
    vec<7> result{};

    for (size_t i = 0; i < 7; ++i)
    {
        result[i][0] =
            static_cast<float>(
                v(static_cast<Eigen::Index>(i))
            );
    }

    return result;
}


vec<6> eigenToVec6(const Vec6E& v)
{
    vec<6> result{};

    for (size_t i = 0; i < 6; ++i)
    {
        result[i][0] =
            static_cast<float>(
                v(static_cast<Eigen::Index>(i))
            );
    }

    return result;
}


Eigen::Vector3d getPosition(const vec<7>& pose)
{
    return Eigen::Vector3d(
        pose[0][0],
        pose[1][0],
        pose[2][0]
    );
}


Eigen::Quaterniond getOrientation(const vec<7>& pose)
{
    // GetPose format:
    // [x y z qw qx qy qz]

    Eigen::Quaterniond q(
        pose[3][0],
        pose[4][0],
        pose[5][0],
        pose[6][0]
    );

    if (!q.coeffs().allFinite())
    {
        throw std::runtime_error(
            "GetPose returned non-finite quaternion"
        );
    }

    if (q.norm() < 1e-10)
    {
        throw std::runtime_error(
            "GetPose returned zero quaternion"
        );
    }

    q.normalize();

    return q;
}



void wrapJoints(vec<7>& q)
{
    for (size_t i = 0; i < 7; ++i)
    {
        while (q[i][0] > PI)
        {
            q[i][0] -= 2.0f * PI;
        }

        while (q[i][0] < -PI)
        {
            q[i][0] += 2.0f * PI;
        }
    }
}




bool isFinite(const vec<7>& v)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (!std::isfinite(v[i][0]))
        {
            return false;
        }
    }

    return true;
}


bool isFinite(const vec<6>& v)
{
    for (size_t i = 0; i < 6; ++i)
    {
        if (!std::isfinite(v[i][0]))
        {
            return false;
        }
    }

    return true;
}



float maxAbs(const vec<7>& v)
{
    float result = 0.0f;

    for (size_t i = 0; i < 7; ++i)
    {
        result =
            std::max(
                result,
                static_cast<float>(
                    std::abs(v[i][0])
                )
            );
    }

    return result;
}


float maxAbs(const vec<6>& v)
{
    float result = 0.0f;

    for (size_t i = 0; i < 6; ++i)
    {
        result =
            std::max(
                result,
                static_cast<float>(
                    std::abs(v[i][0])
                )
            );
    }

    return result;
}



bool poseErrorWarning(const vec<6>& e)
{
    for (size_t i = 0; i < 3; ++i)
    {
        if (std::abs(e[i][0]) >
            cfg.max_shadow_position_error_m)
        {
            return true;
        }
    }

    for (size_t i = 3; i < 6; ++i)
    {
        if (std::abs(e[i][0]) >
            cfg.max_shadow_rotation_error_rad)
        {
            return true;
        }
    }

    return false;
}


bool cartVelocityWarning(const vec<6>& v)
{
    for (size_t i = 0; i < 3; ++i)
    {
        if (std::abs(v[i][0]) >
            cfg.max_shadow_linear_speed_m_s)
        {
            return true;
        }
    }

    for (size_t i = 3; i < 6; ++i)
    {
        if (std::abs(v[i][0]) >
            cfg.max_shadow_angular_speed_rad_s)
        {
            return true;
        }
    }

    return false;
}


bool impedanceTorqueWarning(const vec<7>& tau)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (std::abs(tau[i][0]) >
            cfg.max_shadow_impedance_torque_nm)
        {
            return true;
        }
    }

    return false;
}


// PHYSICAL LIMITS

bool torqueLimitExceeded(const vec<7>& tau)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (std::abs(tau[i][0]) >
            maxTau[i][0])
        {
            return true;
        }
    }

    return false;
}


bool currentLimitExceeded(const vec<7>& current)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (std::abs(current[i][0]) >
            maxCurrent[i][0])
        {
            return true;
        }
    }

    return false;
}


void saturateTorque(vec<7>& tau)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (std::abs(tau[i][0]) >
            maxTau[i][0])
        {
            tau[i][0] =
                std::copysign(
                    maxTau[i][0],
                    tau[i][0]
                );
        }
    }
}


void saveLogger()
{
    logger.save_csv(
        "impedance_shadow_data.csv"
    );

    const auto t =
        std::time(nullptr);

    const auto tm =
        *std::localtime(&t);

    std::ostringstream ss;

    ss << std::put_time(
        &tm,
        "%Y-%m-%d_%H-%M-%S"
    );

    logger.save_csv(
        "impedance_shadow_data_" +
        ss.str() +
        ".csv"
    );

    std::cout
        << "\nData saved:\n"
        << "  impedance_shadow_data.csv\n"
        << "  impedance_shadow_data_"
        << ss.str()
        << ".csv\n";
}


// GRAVITY-ONLY TRANSITION

void sendGravity(
    KinovaRobotLowLevel& robot,
    int cycles)
{
    for (int cycle = 0;
         cycle < cycles &&
         !robot.timeout;
         ++cycle)
    {
        vec<7> q =
            robot.getPos();

        wrapJoints(q);

        if (!isFinite(q))
        {
            throw std::runtime_error(
                "Non-finite q during gravity transition"
            );
        }

        vec<7> gravity =
            GravityVector(q);

        if (!isFinite(gravity))
        {
            throw std::runtime_error(
                "Non-finite gravity torque"
            );
        }

        // Absolute torque barrier.
        saturateTorque(gravity);

        const vec<7> current =
            KTGR.hadamard(gravity);

        if (!isFinite(current))
        {
            throw std::runtime_error(
                "Non-finite gravity current"
            );
        }

        if (currentLimitExceeded(current))
        {
            throw std::runtime_error(
                "Gravity current exceeds maxCurrent"
            );
        }

        robot.sendCur(current);
    }
}



const char* healthString(bool warning)
{
    return warning
        ? "WARNING"
        : "OK";
}

// DASHBOARD

void printDashboard(
    double time,
    double impedance_blend,

    const vec<7>& pose,
    const vec<6>& pose_error,
    const vec<6>& cart_velocity,

    const vec<7>& tau_task,
    const vec<7>& tau_null,
    const vec<7>& tau_ext,
    const vec<7>& tau_imp,

    const vec<7>& gravity,
    const vec<7>& tau_virtual,
    const vec<7>& tau_hw,

    const vec<7>& current_virtual,
    const vec<7>& current_cmd,
    const vec<7>& current_meas,

    bool finite_ok,
    bool pose_warning,
    bool velocity_warning,
    bool impedance_warning,
    bool virtual_tau_warning,
    bool virtual_current_warning)
{
    std::cout
        << "\033[2J\033[H";

    std::cout
        << "============================================================\n"
        << "       KINOVA GEN3 CARTESIAN IMPEDANCE SHADOW TEST\n"
        << "============================================================\n";

    if constexpr (ENABLE_IMPEDANCE_HARDWARE)
    {
        std::cout
            << " MODE : IMPEDANCE + GRAVITY HARDWARE\n";
    }
    else
    {
        std::cout
            << " MODE : SHADOW / VIRTUAL IMPEDANCE\n"
            << " HW   : GRAVITY COMPENSATION ONLY\n";
    }

    std::cout
        << "------------------------------------------------------------\n"
        << " Cartesian task          : ENABLED\n"
        << " Nullspace               : "
        << (cfg.enable_nullspace ? "ENABLED" : "DISABLED")
        << "\n"
        << " Desired wrench          : "
        << (cfg.enable_wrench ? "ENABLED" : "DISABLED")
        << "\n"
        << " Impedance -> Hardware   : "
        << (ENABLE_IMPEDANCE_HARDWARE ? "YES" : "NO")
        << "\n"
        << "------------------------------------------------------------\n"
        << std::fixed
        << std::setprecision(4)
        << " Time                    : "
        << time
        << " s\n"
        << " Hardware blend          : "
        << impedance_blend * 100.0
        << " %\n"
        << "------------------------------------------------------------\n";


    std::cout
        << "\nPOSE [x y z qw qx qy qz]\n"
        << pose
        << "\n";


    std::cout
        << "\nDELTA XI [dx dy dz dRx dRy dRz]\n"
        << pose_error
        << "\n";


    std::cout
        << "\nCARTESIAN VELOCITY J*dq\n"
        << cart_velocity
        << "\n";


    std::cout
        << "\nTAU TASK - VIRTUAL [Nm]\n"
        << tau_task
        << "\n";


    std::cout
        << "\nTAU NULLSPACE - VIRTUAL [Nm]\n"
        << tau_null
        << "\n";


    std::cout
        << "\nTAU EXTERNAL WRENCH - VIRTUAL [Nm]\n"
        << tau_ext
        << "\n";


    std::cout
        << "\nTAU IMPEDANCE TOTAL - VIRTUAL [Nm]\n"
        << "(rate-limited total controller output)\n"
        << tau_imp
        << "\n";


    std::cout
        << "\nGRAVITY TORQUE [Nm]\n"
        << gravity
        << "\n";


    std::cout
        << "\nTOTAL TORQUE - VIRTUAL [Nm]\n"
        << "gravity + tau_imp\n"
        << tau_virtual
        << "\n";


    std::cout
        << "\nTORQUE ACTUALLY USED FOR HARDWARE [Nm]\n"
        << tau_hw
        << "\n";


    std::cout
        << "\nVIRTUAL CURRENT [A]\n"
        << current_virtual
        << "\n";


    std::cout
        << "\nCURRENT ACTUALLY SENT [A]\n"
        << current_cmd
        << "\n";


    std::cout
        << "\nMEASURED CURRENT [A]\n"
        << current_meas
        << "\n";


    std::cout
        << "\n------------------------------------------------------------\n"
        << " SHADOW HEALTH\n"
        << "------------------------------------------------------------\n"
        << " Finite values           : "
        << (finite_ok ? "OK" : "FAIL")
        << "\n"
        << " Pose error              : "
        << healthString(pose_warning)
        << "\n"
        << " Cartesian velocity      : "
        << healthString(velocity_warning)
        << "\n"
        << " Impedance torque        : "
        << healthString(impedance_warning)
        << "\n"
        << " Virtual torque limit    : "
        << healthString(virtual_tau_warning)
        << "\n"
        << " Virtual current limit   : "
        << healthString(virtual_current_warning)
        << "\n";


    std::cout
        << "\n------------------------------------------------------------\n"
        << " MAX VALUES\n"
        << "------------------------------------------------------------\n"
        << " Max |DeltaXi|           : "
        << maxAbs(pose_error)
        << "\n"
        << " Max |J*dq|              : "
        << maxAbs(cart_velocity)
        << "\n"
        << " Max |tau_task|          : "
        << maxAbs(tau_task)
        << " Nm\n"
        << " Max |tau_null|          : "
        << maxAbs(tau_null)
        << " Nm\n"
        << " Max |tau_ext|           : "
        << maxAbs(tau_ext)
        << " Nm\n"
        << " Max |tau_imp|           : "
        << maxAbs(tau_imp)
        << " Nm\n"
        << " Max |tau_virtual|       : "
        << maxAbs(tau_virtual)
        << " Nm\n"
        << " Max |I_virtual|         : "
        << maxAbs(current_virtual)
        << " A\n"
        << " Max |I_HW|              : "
        << maxAbs(current_cmd)
        << " A\n";


    std::cout
        << "\n------------------------------------------------------------\n"
        << " CONTROL PATH\n"
        << "------------------------------------------------------------\n"
        << " Gravity -> Hardware     : YES\n"
        << " Task calculation        : YES\n"
        << " Nullspace calculation   : "
        << (cfg.enable_nullspace ? "YES" : "NO")
        << "\n"
        << " Wrench calculation      : "
        << (cfg.enable_wrench ? "YES" : "NO")
        << "\n"
        << " Impedance -> Hardware   : "
        << (ENABLE_IMPEDANCE_HARDWARE ? "YES" : "NO")
        << "\n"
        << " Data Logger             : YES\n"
        << "------------------------------------------------------------\n"
        << " Ctrl+C -> gravity transition -> restore modes -> save CSV\n"
        << "============================================================\n";

    std::cout.flush();
}


//main

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::unique_ptr<KinovaRobotLowLevel> robot;

    try
    {
        // IO / UDP
        
        boost::asio::io_context io_context;

        udp_server server{
            io_context,
            "KG3"
        };


        // CONNECT
        
        std::cout
            << "\nConnecting to Kinova Gen3 at "
            << ROBOT_IP
            << "...\n";


        robot =
            std::make_unique<KinovaRobotLowLevel>(
                ROBOT_IP,
                USERNAME,
                PASSWORD
            );


        robot->wait_callback_user_data =
            &io_context;


        robot->wait_callback =
            [](void* context)
            {
                static_cast<
                    boost::asio::io_context*
                >(context)->poll_one();
            };


        // INITIAL FEEDBACK
        
        vec<7> q =
            robot->getPos();

        vec<7> dq =
            robot->getVel();

        wrapJoints(q);


        if (!isFinite(q) ||
            !isFinite(dq))
        {
            throw std::runtime_error(
                "Initial q/dq contains NaN/Inf"
            );
        }


        vec<7> last_pose =
            GetPose(q);


        if (!isFinite(last_pose))
        {
            throw std::runtime_error(
                "Initial GetPose contains NaN/Inf"
            );
        }


        const Eigen::Vector3d
            initial_position =
                getPosition(last_pose);


        const Eigen::Quaterniond
            initial_orientation =
                getOrientation(last_pose);


        const Vec7E
            initial_q =
                toEigen7(q);


        // CONTROLLER CONFIGURATION
        
        CartesianImpedanceController controller;


        // Cartesian stiffness
        controller.setCartesianStiffness(
            cfg.stiffness
        );


        // Nullspace stiffness
        controller.setNullspaceStiffness(
            cfg.enable_nullspace
                ? cfg.nullspace_stiffness
                : 0.0
        );


        // Cartesian + nullspace damping
        controller.setDampingFactors(
            cfg.damping_factors,

            cfg.enable_nullspace
                ? cfg.nullspace_damping_factor
                : 0.0
        );


        controller.setFiltering(
            cfg.update_hz,
            cfg.pose_filter,
            cfg.stiffness_filter,
            cfg.wrench_filter,
            cfg.nullspace_filter
        );


        controller.setMaxTorqueDelta(
            cfg.delta_tau_max
        );


        controller.setWrench(
            cfg.enable_wrench
                ? cfg.wrench
                : Vec6E::Zero()
        );


        // DESIRED CARTESIAN REFERENCE
        // By default:
        // desired pose = initial pose
        Eigen::Vector3d desired_position =
            initial_position;

        Eigen::Quaterniond desired_orientation =
            initial_orientation;


        if (cfg.use_cartesian_offset)
        {
            // position_offset is expressed in the ROBOT BASE frame.
            desired_position +=
                cfg.position_offset;


            constexpr double DEG2RAD =
                3.14159265358979323846 / 180.0;


            const double roll =
                cfg.orientation_offset_rpy_deg(0)
                * DEG2RAD;

            const double pitch =
                cfg.orientation_offset_rpy_deg(1)
                * DEG2RAD;

            const double yaw =
                cfg.orientation_offset_rpy_deg(2)
                * DEG2RAD;


            Eigen::Quaterniond orientation_offset =
                Eigen::AngleAxisd(
                    yaw,
                    Eigen::Vector3d::UnitZ()
                )
                *
                Eigen::AngleAxisd(
                    pitch,
                    Eigen::Vector3d::UnitY()
                )
                *
                Eigen::AngleAxisd(
                    roll,
                    Eigen::Vector3d::UnitX()
                );


            // Apply rotation relative to the INITIAL EE orientation.
            //
            // R_d = R_0 * R_offset
            
            desired_orientation =
                initial_orientation
                *
                orientation_offset;


            desired_orientation.normalize();
        }


        // INITIALIZE CONTROLLER WITH DESIRED REFERENCE
        
        controller.initialize(
            desired_position,
            desired_orientation,
            initial_q
        );
        

        Vec7E nullspace_reference =
            initial_q;


        if (cfg.enable_nullspace)
        {
            nullspace_reference +=
                cfg.nullspace_offset;
        }


        controller.setNullspaceReference(
            nullspace_reference
        );


        
        std::cout
            << "\n============================================================\n"
            << " CONTROLLER CONFIGURATION\n"
            << "============================================================\n"
            << " Cartesian task    : ENABLED\n"
            << " Nullspace         : "
            << (cfg.enable_nullspace
                    ? "ENABLED"
                    : "DISABLED")
            << "\n"
            << " Desired wrench    : "
            << (cfg.enable_wrench
                    ? "ENABLED"
                    : "DISABLED")
            << "\n"
            << " Impedance -> HW   : "
            << (ENABLE_IMPEDANCE_HARDWARE
                    ? "YES"
                    : "NO")
            << "\n"
            << " Gravity -> HW     : YES\n"
            << "============================================================\n";


        std::cout
            << "\nStiffness:\n"
            << cfg.stiffness.transpose()
            << "\n";


        std::cout
            << "\nDamping factors:\n"
            << cfg.damping_factors.transpose()
            << "\n";


        if (cfg.enable_nullspace)
        {
            std::cout
                << "\nNullspace stiffness: "
                << cfg.nullspace_stiffness
                << "\n"
                << "Nullspace offset [rad]:\n"
                << cfg.nullspace_offset.transpose()
                << "\n";
        }


        if (cfg.enable_wrench)
        {
            std::cout
                << "\nDesired wrench:\n"
                << cfg.wrench.transpose()
                << "\n";
        }


        // CURRENT MODE
        
        using enum kinova_robot::ControlMode;


        robot->setControlMode(
            {
                CUR,
                CUR,
                CUR,
                CUR,
                CUR,
                CUR,
                CUR
            }
        );


        // GRAVITY STARTUP

        std::cout
            << "\nStarting gravity compensation...\n";


        sendGravity(
            *robot,
            cfg.gravity_warmup_cycles
        );


        std::cout
            << "Gravity compensation active.\n"
            << "Starting shadow experiment.\n";


        // CLOCK

        using Clock =
            std::chrono::steady_clock;


        const auto experiment_start =
            Clock::now();


        auto previous_loop_time =
            experiment_start;


        double last_display_time =
            -cfg.dashboard_period_s;


        double impedance_blend =
            0.0;


        // CONTROL LOOP

        while (
            !stop_requested &&
            !robot->timeout
        )
        {
            const auto now =
                Clock::now();


            const double time =
                std::chrono::duration<double>(
                    now -
                    experiment_start
                ).count();


            const double dt =
                std::chrono::duration<double>(
                    now -
                    previous_loop_time
                ).count();


            previous_loop_time =
                now;


            if (
                cfg.experiment_duration_s > 0.0
                &&
                time >=
                    cfg.experiment_duration_s
            )
            {
                break;
            }


            // 1. ROBOT FEEDBACK

            q =
                robot->getPos();

            dq =
                robot->getVel();

            wrapJoints(q);


            const vec<7>
                current_meas =
                    robot->getCur();


            const vec<7>
                tau_meas =
                    robot->getTau();


            if (!isFinite(q) ||
                !isFinite(dq) ||
                !isFinite(current_meas) ||
                !isFinite(tau_meas))
            {
                throw std::runtime_error(
                    "Non-finite robot feedback"
                );
            }


            // 2. ANALYTIC MODEL

            const vec<7> pose =
                GetPose(
                    q,
                    last_pose
                );


            last_pose =
                pose;


            const mat<6, 7> J_lab =
                GetJacobian(q);


            const vec<7> gravity =
                GravityVector(q);


            if (!isFinite(pose) ||
                !isFinite(gravity))
            {
                throw std::runtime_error(
                    "Non-finite analytic model output"
                );
            }


            // 3. CONVERT TO EIGEN FIXED-SIZE TYPES

            const Vec7E q_eigen =
                toEigen7(q);


            const Vec7E dq_eigen =
                toEigen7(dq);


            const Mat67E J_eigen =
                toEigenJacobian(J_lab);


            const Eigen::Vector3d position =
                getPosition(pose);


            const Eigen::Quaterniond orientation =
                getOrientation(pose);


            if (!q_eigen.allFinite() ||
                !dq_eigen.allFinite() ||
                !J_eigen.allFinite() ||
                !position.allFinite() ||
                !orientation.coeffs().allFinite())
            {
                throw std::runtime_error(
                    "Non-finite Eigen state"
                );
            }


            // 4. FULL SHADOW IMPEDANCE CONTROLLER
            //
            // compute() returns:
            //
            // tau_task
            // tau_null
            // tau_ext
            // tau_impedance
            // pose_error
            
            const auto ctrl =
                controller.compute(
                    q_eigen,
                    dq_eigen,
                    position,
                    orientation,
                    J_eigen
                );


            if (!ctrl.tau_task.allFinite() ||
                !ctrl.tau_null.allFinite() ||
                !ctrl.tau_ext.allFinite() ||
                !ctrl.tau_impedance.allFinite() ||
                !ctrl.pose_error.allFinite())
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: controller returned NaN/Inf"
                );
            }


            const vec<7> tau_task =
                eigenToVec7(
                    ctrl.tau_task
                );


            const vec<7> tau_null =
                eigenToVec7(
                    ctrl.tau_null
                );


            const vec<7> tau_ext =
                eigenToVec7(
                    ctrl.tau_ext
                );


            const vec<7> tau_imp =
                eigenToVec7(
                    ctrl.tau_impedance
                );


            const vec<6> pose_error =
                eigenToVec6(
                    ctrl.pose_error
                );


            // 5. CARTESIAN VELOCITY
            
            const Vec6E
                cart_velocity_eigen =
                    J_eigen *
                    dq_eigen;


            if (!cart_velocity_eigen.allFinite())
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: J*dq contains NaN/Inf"
                );
            }


            const vec<6>
                cart_velocity =
                    eigenToVec6(
                        cart_velocity_eigen
                    );


            // 6. VIRTUAL TOTAL TORQUE
            
            const vec<7>
                tau_virtual =
                    gravity +
                    tau_imp;


            if (!isFinite(tau_virtual))
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: tau_virtual contains NaN/Inf"
                );
            }


            // 7. VIRTUAL CURRENT

            const vec<7>
                current_virtual =
                    KTGR.hadamard(
                        tau_virtual
                    );


            if (!isFinite(current_virtual))
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: current_virtual contains NaN/Inf"
                );
            }


            // 8. SHADOW HEALTH

            const bool finite_ok =
                isFinite(pose_error)
                &&
                isFinite(cart_velocity)
                &&
                isFinite(tau_task)
                &&
                isFinite(tau_null)
                &&
                isFinite(tau_ext)
                &&
                isFinite(tau_imp)
                &&
                isFinite(tau_virtual);


            if (!finite_ok)
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: non-finite controller signal"
                );
            }


            const bool pose_warning =
                poseErrorWarning(
                    pose_error
                );


            const bool velocity_warning =
                cartVelocityWarning(
                    cart_velocity
                );


            const bool impedance_warning =
                impedanceTorqueWarning(
                    tau_imp
                );


            const bool virtual_tau_warning =
                torqueLimitExceeded(
                    tau_virtual
                );


            const bool virtual_current_warning =
                currentLimitExceeded(
                    current_virtual
                );


            // 9. HARDWARE TORQUE

            vec<7> tau_hw{};


            if constexpr (
                ENABLE_IMPEDANCE_HARDWARE)
            {

                if (
                    cfg.impedance_blend_time_s
                    >
                    0.0
                )
                {
                    impedance_blend +=
                        dt /
                        cfg.impedance_blend_time_s;
                }
                else
                {
                    impedance_blend =
                        1.0;
                }


                impedance_blend =
                    std::clamp(
                        impedance_blend,
                        0.0,
                        1.0
                    );


                tau_hw =
                    gravity;


                for (size_t joint = 0;
                     joint < 7;
                     ++joint)
                {
                    tau_hw[joint][0] +=
                        static_cast<float>(
                            impedance_blend
                        )
                        *
                        tau_imp[joint][0];
                }


                saturateTorque(
                    tau_hw
                );
            }
            else
            {
                // SHADOW MODE
                //
                // ONLY GRAVITY GOES TO HARDWARE.

                impedance_blend =
                    0.0;


                tau_hw =
                    gravity;
            }


            if (!isFinite(tau_hw))
            {
                throw std::runtime_error(
                    "HARDWARE SAFETY: tau_hw contains NaN/Inf"
                );
            }


            // 10. TORQUE -> CURRENT

            const vec<7>
                current_cmd =
                    KTGR.hadamard(
                        tau_hw
                    );


            if (!isFinite(current_cmd))
            {
                throw std::runtime_error(
                    "HARDWARE SAFETY: current_cmd contains NaN/Inf"
                );
            }


            // 11. REAL HARDWARE CURRENT BARRIER

            if (
                currentLimitExceeded(
                    current_cmd
                )
            )
            {
                throw std::runtime_error(
                    "HARDWARE SAFETY: current command exceeds maxCurrent"
                );
            }


            // 12. SEND TO ROBOT

            robot->sendCur(
                current_cmd
            );


            // 13. LOGGER

            logger.log(

                "t"_nm =
                    static_cast<float>(
                        time
                    ),

                "q"_nm =
                    q,

                "dq"_nm =
                    dq,

                "pose"_nm =
                    pose,

                "pose_err"_nm =
                    pose_error,

                "cart_vel"_nm =
                    cart_velocity,

                "tau_task"_nm =
                    tau_task,

                "tau_null"_nm =
                    tau_null,

                "tau_ext"_nm =
                    tau_ext,

                "tau_imp"_nm =
                    tau_imp,

                "tau_g"_nm =
                    gravity,

                "tau_virtual"_nm =
                    tau_virtual,

                "tau_hw"_nm =
                    tau_hw,

                "current_virtual"_nm =
                    current_virtual,

                "current_cmd"_nm =
                    current_cmd,

                "current_meas"_nm =
                    current_meas,

                "tau_meas"_nm =
                    tau_meas
            );



            if (
                time -
                last_display_time
                >=
                cfg.dashboard_period_s
            )
            {
                last_display_time =
                    time;


                printDashboard(

                    time,

                    impedance_blend,

                    pose,

                    pose_error,

                    cart_velocity,

                    tau_task,

                    tau_null,

                    tau_ext,

                    tau_imp,

                    gravity,

                    tau_virtual,

                    tau_hw,

                    current_virtual,

                    current_cmd,

                    current_meas,

                    finite_ok,

                    pose_warning,

                    velocity_warning,

                    impedance_warning,

                    virtual_tau_warning,

                    virtual_current_warning
                );
            }
        }



        std::cout
            << "\n\nStopping experiment...\n"
            << "Returning to gravity-only transition...\n";


        sendGravity(
            *robot,
            cfg.gravity_shutdown_cycles
        );


        robot.reset();


        saveLogger();


        std::cout
            << "\nExperiment finished cleanly.\n";


        return 0;
    }


    // ========================================================
    // STD EXCEPTION
    // ========================================================

    catch (const std::exception& error)
    {
        std::cerr
            << "\n\n"
            << "============================================================\n"
            << " SAFETY / EXCEPTION EXIT\n"
            << "============================================================\n"
            << error.what()
            << "\n";


        if (robot)
        {
            try
            {
                std::cerr
                    << "Attempting gravity-only transition...\n";


                sendGravity(
                    *robot,
                    cfg.gravity_shutdown_cycles
                );
            }
            catch (
                const std::exception&
                gravity_error
            )
            {
                std::cerr
                    << "Gravity transition failed: "
                    << gravity_error.what()
                    << "\n";
            }
            catch (...)
            {
                std::cerr
                    << "Gravity transition failed "
                    << "with unknown exception.\n";
            }


            robot.reset();
        }


        try
        {
            saveLogger();
        }
        catch (...)
        {
            std::cerr
                << "Could not save CSV after exception.\n";
        }


        return 1;
    }


    // ========================================================
    // UNKNOWN EXCEPTION
    // ========================================================

    catch (...)
    {
        std::cerr
            << "\nUnknown fatal exception.\n";


        if (robot)
        {
            try
            {
                sendGravity(
                    *robot,
                    cfg.gravity_shutdown_cycles
                );
            }
            catch (...)
            {
            }


            robot.reset();
        }


        try
        {
            saveLogger();
        }
        catch (...)
        {
        }


        return 1;
    }
}
