/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <numbers>
#include <algorithm>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <phoenix_ros_driver/msg/talon_ctrl.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>

// #include "util/pub_map.hpp"
#include "util/ros_utils.hpp"
#include "robot/robot_math.hpp"


using namespace util;
using namespace util::ros_aliases;
using namespace std::chrono;
using namespace std::chrono_literals;

using Int32Msg = std_msgs::msg::Int32;
using TwistMsg = geometry_msgs::msg::Twist;
using Float64Msg = std_msgs::msg::Float64;
using OdometryMsg = nav_msgs::msg::Odometry;
using JointStateMsg = sensor_msgs::msg::JointState;

using TalonCtrlMsg = phoenix_ros_driver::msg::TalonCtrl;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;
using TalonFaultsMsg = phoenix_ros_driver::msg::TalonFaults;


#define SIM_STEP_DT_MS 1
#define IO_PUB_DT_MS   20

#define ROBOT_TOPIC(topic) "lance/" topic
#define WATCHDOG_TOPIC     ROBOT_TOPIC("watchdog_status")

#define GZ_LEFT_TRACK_ODOM_TOPIC  "/left_track_odom"
#define GZ_RIGHT_TRACK_ODOM_TOPIC "/right_track_odom"
#define GZ_HOPPER_VEL_TOPIC       "/dump_cmd_vel"
#define GZ_TWIST_TOPIC            "/cmd_vel"
#define GZ_JOINT_STATES_TOPIC     "/gz_joint_states"

#define HOPPER_LINK_NAME "hopper_joint"


// velocities in radians per second
static void motorVelsToRobotTwist(double l, double r, TwistMsg& twist)
{
    constexpr double RATIO =
        (1. / lance::TRACK_GEARING) * (lance::TRACK_EFFECTIVE_OUTPUT_RADIUS_M);

    double v_l = l * RATIO;
    double v_r = r * RATIO;

    twist.linear.x = (v_r + v_l) / 2.0;
    twist.angular.z = (v_r - v_l) / lance::TRACK_SEPARATION_M;
}


// --- BATTERY MODEL -----------------------------------------------------------

class SimpleBattery
{
public:
    SimpleBattery(
        double nominal_voltage = 16.0,
        double internal_resistance = 0.02);

public:
    double voltage(double total_current);

private:
    const double nominal_voltage;
    const double internal_resistance;
};


SimpleBattery::SimpleBattery(
    double nominal_voltage,
    double internal_resistance) :
    nominal_voltage{nominal_voltage},
    internal_resistance{internal_resistance}
{
}

double SimpleBattery::voltage(double total_current)
{
    return std::max(
        0.0,
        this->nominal_voltage - (total_current * this->internal_resistance));
}



// --- MOTOR MODEL -------------------------------------------------------------

class BasicMotor
{
public:
    BasicMotor(
        double inertia = 5e-5,
        double kv_rpm_per_volt = 531.7,
        double kt_nm_per_amp = 0.0184,
        double stall_current = 257.,
        double resistance_ohm = (12. / 257.),
        double damping = 0.002);

public:
    void setEnabled(bool e);
    void setControl(const TalonCtrlMsg& ctrl);
    void fillInfo(TalonInfoMsg& info, double bus_voltage);

    void step(double dt, double bus_voltage);

    double getVelocity() const;
    double getSupplyCurrent() const;

private:
    const double kv_rpm_per_volt;
    const double kt_nm_per_amp;
    const double stall_current;
    const double resistance;
    const double inertia;
    const double damping;

    double position{0.};
    double velocity{0.};
    double acceleration{0};

    double output_voltage{0.};
    double output_current{0.};
    double output_percent{0.};
    double supply_current{0.};

    TalonCtrlMsg setpoint{};
    bool enabled{true};
};

BasicMotor::BasicMotor(
    double inertia,
    double kv_rpm_per_volt,
    double kt_nm_per_amp,
    double stall_current,
    double resistance_ohm,
    double damping) :
    kv_rpm_per_volt(kv_rpm_per_volt),
    kt_nm_per_amp(kt_nm_per_amp),
    stall_current(stall_current),
    resistance(resistance_ohm),
    inertia(inertia),
    damping(damping)
{
}

void BasicMotor::setEnabled(bool e) { this->enabled = e; }

void BasicMotor::setControl(const TalonCtrlMsg& ctrl) { this->setpoint = ctrl; }

void BasicMotor::fillInfo(TalonInfoMsg& info, double bus_voltage)
{
    info.position = this->position / (2.0 * std::numbers::pi);  // turns
    info.velocity = this->velocity / (2.0 * std::numbers::pi);  // turns/s
    info.acceleration =
        this->acceleration / (2.0 * std::numbers::pi);  // turns/s^2
    info.device_temp = 0.f;
    info.processor_temp = 0.f;
    info.bus_voltage = static_cast<float>(bus_voltage);
    info.output_percent = static_cast<float>(this->output_percent);
    info.output_voltage = static_cast<float>(this->output_voltage);
    info.output_current = static_cast<float>(this->output_current);
    info.supply_current = static_cast<float>(this->supply_current);
    info.status = this->enabled ? 0b11 : 0b10;
    info.control_mode = this->setpoint.mode;
}

void BasicMotor::step(double dt, double bus_voltage)
{
    double applied_voltage = 0.0;
    double effort = 0.0;

    if (!enabled || this->setpoint.mode == TalonCtrlMsg::DISABLED)
    {
        applied_voltage = 0.0;
        effort = 0.0;
    }
    else
    {
        switch (this->setpoint.mode)
        {
            case TalonCtrlMsg::PERCENT_OUTPUT:
            {
                effort = std::clamp(this->setpoint.value, -1.0, 1.0);
                applied_voltage = effort * bus_voltage;
                break;
            }
            case TalonCtrlMsg::VOLTAGE:
            {
                applied_voltage =
                    std::clamp(this->setpoint.value, -bus_voltage, bus_voltage);
                effort = applied_voltage / bus_voltage;
                break;
            }
            case TalonCtrlMsg::VELOCITY:
            {
                // turns/s -> rad/s
                const double vel_sp_rad_s =
                    this->setpoint.value * 2.0 * std::numbers::pi;
                const double kv_rad_s_per_volt =
                    this->kv_rpm_per_volt * (2.0 * std::numbers::pi / 60.0);
                const double vel_error = vel_sp_rad_s - this->velocity;

                // TODO: full PID
                // how much voltage is needed according to the model
                const double kf_volts = vel_sp_rad_s / kv_rad_s_per_volt;
                // p-gain
                const double kp_volts = vel_error * 0.2;

                applied_voltage =
                    std::clamp(kf_volts + kp_volts, -bus_voltage, bus_voltage);
                effort = applied_voltage / bus_voltage;
                break;
            }
            default:
            {
                applied_voltage = 0.0;
                effort = 0.0;
                break;
            }
        }
    }

    // Back-EMF
    const double kv_rad_s_per_volt =
        kv_rpm_per_volt * (2.0 * std::numbers::pi / 60.0);
    const double back_emf = (this->velocity / kv_rad_s_per_volt);
    const double voltage_diff = (applied_voltage - back_emf);

    // Current
    this->output_current = std::clamp(
        (std::abs(this->resistance) > 1e-9) ? voltage_diff / this->resistance
                                            : 0.0,
        -this->stall_current,
        this->stall_current);

    // Torque
    const double torque = (kt_nm_per_amp * output_current);
    const double torque_net = torque - (damping * velocity);

    // Dynamics
    this->acceleration = torque_net / inertia;
    this->velocity += acceleration * dt;
    this->position += velocity * dt;

    // Outputs
    this->output_voltage = applied_voltage;
    this->output_percent = effort;

    // Match input<->output power, invert if back-emf is the dominant voltage
    this->supply_current =
        std::abs((this->output_voltage * this->output_current) / bus_voltage) *
        (voltage_diff < 0. ? -1. : 1.);
}

double BasicMotor::getVelocity() const { return this->velocity; }

double BasicMotor::getSupplyCurrent() const { return this->supply_current; }



// --- ACTUATOR MODEL ----------------------------------------------------------

class BasicActuator
{
public:
    BasicActuator(double max_speed = 0.2, double init_position = 0.5);

public:
    void setEnabled(bool e);
    void setControl(const TalonCtrlMsg& msg);
    void fillInfo(TalonInfoMsg& info, double bus_voltage);

    void step(double dt);

    void setPosition(double p);
    double getPosition() const;

private:
    const double max_speed;

    double position{0.};
    double velocity{0.};
    double output_percent{0.};

    TalonCtrlMsg setpoint;
    bool enabled{true};
};

BasicActuator::BasicActuator(double max_speed, double init_position) :
    max_speed(max_speed),
    position(init_position)
{
}

void BasicActuator::setEnabled(bool e) { this->enabled = e; }

void BasicActuator::setControl(const TalonCtrlMsg& msg)
{
    this->setpoint = msg;
}

void BasicActuator::fillInfo(TalonInfoMsg& info, double bus_voltage)
{
    info.position = this->position * 1000.;
    info.velocity = this->velocity;
    info.acceleration = 0.0;
    info.device_temp = 0.f;
    info.processor_temp = 0.f;
    info.bus_voltage = static_cast<float>(bus_voltage);
    info.supply_current = 0.0f;
    info.output_percent = static_cast<float>(this->output_percent);
    info.output_voltage =
        static_cast<float>(this->output_percent * bus_voltage);
    info.output_current = 0.0f;
    info.status = this->enabled ? 0b11 : 0b10;
    info.control_mode = this->setpoint.mode;
}

void BasicActuator::step(double dt)
{
    if (!this->enabled || this->setpoint.mode == TalonCtrlMsg::DISABLED)
    {
        this->velocity = 0.0;
        this->output_percent = 0.0;
    }
    else if (
        this->setpoint.mode == TalonCtrlMsg::PERCENT_OUTPUT ||
        this->setpoint.mode == TalonCtrlMsg::VOLTAGE)
    {
        this->output_percent = std::clamp(this->setpoint.value, -1.0, 1.0);
        this->velocity = this->output_percent * this->max_speed;
    }
    else if (this->setpoint.mode == TalonCtrlMsg::VELOCITY)
    {
        this->velocity =
            std::clamp(this->setpoint.value, -this->max_speed, this->max_speed);
        this->output_percent = this->velocity / this->max_speed;
    }

    position = std::clamp((position + velocity * dt), 0.0, 1.0);
}

void BasicActuator::setPosition(double p) { this->position = p; }

double BasicActuator::getPosition() const { return this->position; }



// --- SIM NODE ----------------------------------------------------------------

class MotorSimNode : public RclNode
{
    struct RclSimMotor
    {
        BasicMotor motor;
        SharedSub<TalonCtrlMsg> ctrl_sub;
        SharedPub<TalonInfoMsg> info_pub;
        // faults pub
    };
    struct RclSimActuator
    {
        BasicActuator actuator;
        SharedSub<TalonCtrlMsg> ctrl_sub;
        SharedPub<TalonInfoMsg> info_pub;
        // faults pub
    };

public:
    MotorSimNode();

private:
    void onWatchdog(int32_t status);

    void iterateSim();
    void publishState();

private:
    std::vector<RclSimMotor> motors;
    std::vector<RclSimActuator> actuators;

    SimpleBattery battery;

    SharedSub<Int32Msg> watchdog_sub;

    SharedSub<JointStateMsg> gz_joint_sub;
    SharedSub<OdometryMsg> left_track_odom_sub;
    SharedSub<OdometryMsg> right_track_odom_sub;
    SharedPub<Float64Msg> act_vel_pub;
    SharedPub<TwistMsg> track_twist_pub;

    RclTimer sim_timer;
    RclTimer io_timer;

    double last_bus_voltage{16.};
    double gz_left_track_pos{0.};
    double gz_left_track_vel{0.};
    double gz_right_track_pos{0.};
    double gz_right_track_vel{0.};
    bool use_gz_track_feedback{false};
};



MotorSimNode::MotorSimNode() :
    RclNode("motor_sim"),
    battery{16.0, 0.01},
    watchdog_sub{this->create_subscription<Int32Msg>(
        WATCHDOG_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& msg) { this->onWatchdog(msg.data); })},
    act_vel_pub{this->create_publisher<Float64Msg>(
        GZ_HOPPER_VEL_TOPIC,
        rclcpp::SystemDefaultsQoS())},
    track_twist_pub{this->create_publisher<TwistMsg>(
        GZ_TWIST_TOPIC,
        rclcpp::SystemDefaultsQoS())},
    sim_timer{this->create_wall_timer(
        milliseconds(SIM_STEP_DT_MS),
        [this]() { this->iterateSim(); })},
    io_timer{this->create_wall_timer(
        milliseconds(IO_PUB_DT_MS),
        [this]() { this->publishState(); })},
    use_gz_track_feedback{
        declare_and_get_param(*this, "use_gz_track_feedback", false)}
{
    constexpr std::array<const char*, 4> MOTOR_NAMES{
        "track_left",
        "track_right",
        "trencher",
        "hopper_belt"};
    constexpr std::array<const char*, 1> ACTUATOR_NAMES{"hopper_act"};

    this->motors.resize(MOTOR_NAMES.size());
    for (size_t i = 0; i < MOTOR_NAMES.size(); i++)
    {
        auto n = MOTOR_NAMES[i];
        auto& m = this->motors[i];

        m.ctrl_sub = this->create_subscription<TalonCtrlMsg>(
            (ROBOT_TOPIC() + std::string(n) + "/ctrl"),
            rclcpp::SystemDefaultsQoS{},
            [this, i](const TalonCtrlMsg& msg)
            { this->motors[i].motor.setControl(msg); });
        m.info_pub = this->create_publisher<TalonInfoMsg>(
            (ROBOT_TOPIC() + std::string(n) + "/info"),
            rclcpp::SensorDataQoS{});
    }

    this->actuators.resize(ACTUATOR_NAMES.size());
    for (size_t i = 0; i < ACTUATOR_NAMES.size(); i++)
    {
        auto n = ACTUATOR_NAMES[i];
        auto& a = this->actuators[i];

        a.actuator.setPosition(0.75);

        a.ctrl_sub = this->create_subscription<TalonCtrlMsg>(
            (ROBOT_TOPIC() + std::string(n) + "/ctrl"),
            rclcpp::SystemDefaultsQoS{},
            [this, i](const TalonCtrlMsg& msg)
            { this->actuators[i].actuator.setControl(msg); });
        a.info_pub = this->create_publisher<TalonInfoMsg>(
            (ROBOT_TOPIC() + std::string(n) + "/info"),
            rclcpp::SensorDataQoS{});
    }

    gz_joint_sub = this->create_subscription<JointStateMsg>(
        GZ_JOINT_STATES_TOPIC,
        rclcpp::SystemDefaultsQoS(),
        [this](const JointStateMsg& msg)
        {
            for (size_t i = 0; i < msg.name.size(); i++)
            {
                if (msg.name[i] == HOPPER_LINK_NAME)
                {
                    double target = lance::linearActuatorToJointAngle(
                        this->actuators[0].actuator.getPosition());
                    double error = target - msg.position[i];
                    if (std::abs(error) > 0.02)
                    {
                        this->act_vel_pub->publish(
                            Float64Msg{}.set__data(
                                std::signbit(error) ? -0.5 : 0.5));
                    }
                    else
                    {
                        this->act_vel_pub->publish(Float64Msg{}.set__data(0.0));
                    }
                    break;
                }
            }
        });
    left_track_odom_sub = this->create_subscription<OdometryMsg>(
        GZ_LEFT_TRACK_ODOM_TOPIC,
        rclcpp::SystemDefaultsQoS(),
        [this](const OdometryMsg& msg)
        {
            this->gz_left_track_pos =
                lance::groundMpsToTrackMotorRps(msg.pose.pose.position.x);
            this->gz_left_track_vel =
                lance::groundMpsToTrackMotorRps(msg.twist.twist.linear.x);
        });
    right_track_odom_sub = this->create_subscription<OdometryMsg>(
        GZ_RIGHT_TRACK_ODOM_TOPIC,
        rclcpp::SystemDefaultsQoS(),
        [this](const OdometryMsg& msg)
        {
            this->gz_right_track_pos =
                lance::groundMpsToTrackMotorRps(msg.pose.pose.position.x);
            this->gz_right_track_vel =
                lance::groundMpsToTrackMotorRps(msg.twist.twist.linear.x);
        });

    RCLCPP_DEBUG(this->get_logger(), "Motor sim started!");
}

void MotorSimNode::onWatchdog(int32_t status)
{
    const bool enable = (status != 0);
    for (auto& m : this->motors)
    {
        m.motor.setEnabled(enable);
    }
    for (auto& a : this->actuators)
    {
        a.actuator.setEnabled(enable);
    }
}

void MotorSimNode::iterateSim()
{
    double dt = (SIM_STEP_DT_MS / 1000.);
    double total_current = 0.0;

    for (auto& m : this->motors)
    {
        m.motor.step(dt, last_bus_voltage);
        total_current += std::abs(m.motor.getSupplyCurrent());
    }
    for (auto& a : this->actuators)
    {
        a.actuator.step(dt);
    }

    last_bus_voltage = battery.voltage(total_current);
}

void MotorSimNode::publishState()
{
    TalonInfoMsg info_msg;
    info_msg.header.stamp = this->get_clock()->now();
    for (size_t i = 0; i < this->motors.size(); i++)
    {
        auto& m = this->motors[i];

        m.motor.fillInfo(info_msg, this->last_bus_voltage);
        // replace track motor states with info from gz (pos, vel)
        if (this->use_gz_track_feedback)
        {
            switch (i)
            {
                case 0:
                {
                    info_msg.position = gz_left_track_pos;
                    info_msg.velocity = gz_left_track_vel;
                    break;
                }
                case 1:
                {
                    info_msg.position = gz_right_track_pos;
                    info_msg.velocity = gz_right_track_vel;
                    break;
                }
            }
        }
        m.info_pub->publish(info_msg);
    }
    for (auto& a : this->actuators)
    {
        a.actuator.fillInfo(info_msg, this->last_bus_voltage);
        a.info_pub->publish(info_msg);
    }

    TwistMsg twist_msg;
    motorVelsToRobotTwist(
        motors[0].motor.getVelocity(),
        motors[1].motor.getVelocity(),
        twist_msg);
    this->track_twist_pub->publish(twist_msg);
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
