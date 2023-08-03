#include "rclcpp/rclcpp.hpp"
#include "pfcbf.h"
#include <random>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>

// For transform support
#include "tf2/convert.h"
#include "tf2/utils.h"

using rcl_interfaces::msg::ParameterType;

class PFCBFNode : public rclcpp::Node {
private:
    // parameter struct
    parameters* params;

    // PFCBF controller
    pfcbf* controller;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control;
    // rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_belief;

    // Subscriber
    rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr sub_pf;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_control;

    // rviz publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pf_marker;
    visualization_msgs::msg::MarkerArray marker_array_msg;

    // polygon visualization
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr circle_viz_;
    int num_points = 0;
    std::vector<double> obstacle_pos;
    double obstacle_radius;

    // Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_safeset_service_;

    // Dynamic parameters handler
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

public:
    bool defined_safeset = false;
    // constructor
    PFCBFNode(const rclcpp::NodeOptions & options) : Node("PFCBFNode", options) {
        

        // Initialize parameters
        this->declare_parameter("alpha", rclcpp::ParameterValue(0.2));
        this->declare_parameter("delta", rclcpp::ParameterValue(0.05));
        this->declare_parameter("b_max", rclcpp::ParameterValue(0.01));
        this->declare_parameter("d", rclcpp::ParameterValue(0.05));
        this->declare_parameter("r_robot", rclcpp::ParameterValue(0.1));
        
        // Default parameters
        params = new parameters();
        params->obstacle_pos = {1., 1., 0.};
        params->obstacle_radius = 0.3;
        params->alpha = 0.2;
        params->delta = 0.05;
        params->nx = 3;
        params->nu = 2;
        params->b_max = 0.01;
        params->r_robot = 0.1;
        params->d = 0.05;
        params->input_bounds = false;
        params->use_orientation_CBF = true;

        // initialize some random particle belief
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(-1.0f, 1.0f);

        belief init_belief;
        for (int i = 0; i < 100; ++i) {
            MyPose pose;
            for (int j = 0; j < 3; ++j) {
                pose[j] = dist(gen);
            }
            init_belief.push_back(pose);
        }

        // initialize PFCBF
        controller = new pfcbf(params, init_belief);

        // publishers
        pub_control = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        // pub_belief = this->create_publisher<geometry_msgs::msg::PoseArray>("pfcbf/particle_cloud", 1);

        // subscribers
        rclcpp::QoS qos(10); // Set the history depth to 10 (modify this according to your use case)
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // Set the reliability to BEST_EFFORT

        sub_pf = this->create_subscription<nav2_msgs::msg::ParticleCloud>("/particle_cloud", qos,
            std::bind(&PFCBFNode::pf_callback, this, std::placeholders::_1));
        sub_control = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_ref", 1,
            std::bind(&PFCBFNode::control_callback, this, std::placeholders::_1));

        // rviz publishers
        pf_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("pf_marker", 1);

        // polygon visualization
        point_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 1,
            std::bind(&PFCBFNode::point_callback, this, std::placeholders::_1));
        circle_viz_ = this->create_publisher<visualization_msgs::msg::Marker>("circle_marker", 1);

        // services
        reset_safeset_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset_safeset",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
                // Callback function to reset safeset
                defined_safeset = false;
                num_points = 0;
            });

        // parameters
        // Add callback for dynamic parameters
        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(
            &PFCBFNode::dynamicParametersCallback,
            this, std::placeholders::_1));

        }

    // Destructor
    ~PFCBFNode() {
        std::cout << "Node deleted!" << std::endl;
    }

    void pf_callback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg) {
        belief pf_belief;
        for (const auto & p : msg->particles) {
            MyPose pose;
            pose[0] = p.pose.position.x;
            pose[1] = p.pose.position.y;
            pose[2] = tf2::getYaw(p.pose.orientation);
            pf_belief.push_back(pose);
        }
        controller->updateParticleBelief(pf_belief);
    }

    void control_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // reference control: u = [v, w]
        VectorXd u_ref(2);
        u_ref << msg->linear.x, msg->angular.z;

        controller->set_reference(u_ref);
    }

    void runControl() {
        // generate safe controls
        VectorXd u_safe = controller->get_control();

        // publish safe control msg
        geometry_msgs::msg::Twist safe_control;
        safe_control.linear.x = u_safe(0);
        safe_control.linear.y = 0.;
        safe_control.linear.z = 0.;
        safe_control.angular.x = 0.;
        safe_control.angular.y = 0.;
        safe_control.angular.z = u_safe(1);
        pub_control->publish(safe_control);
    }

    void propagateBelief() {
        controller->propagateParticles();
    }

    void publish_markers() {
        if (defined_safeset){
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map"; // Set the frame ID according to your coordinate system
                // marker.header.stamp = ros::Time::now();
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.pose.position.x = obstacle_pos[0];
                marker.pose.position.y = obstacle_pos[1];
                marker.pose.position.z = 0.;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 2 * obstacle_radius; // Sphere diameter
                marker.scale.y = 2 * obstacle_radius;
                marker.scale.z = 0.0001;
                marker.color.a = 0.3; // Alpha (transparency)
                marker.color.r = 0.0; // Red
                marker.color.g = 0.0; // Green
                marker.color.b = 1.0; // Blue
                circle_viz_->publish(marker);
            }
    }

    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        cout << "In Point Clicke Callback!" << endl;
        num_points++;
        if (num_points == 1) {
            obstacle_pos = {msg->point.x, msg->point.y};
        }
        else if (num_points == 2) {
            obstacle_radius = std::sqrt(std::pow(msg->point.x - obstacle_pos[0], 2) +
                                        std::pow(msg->point.y - obstacle_pos[1], 2));
            (params->obstacle_pos) = obstacle_pos;
            (params->obstacle_radius) = obstacle_radius;
            defined_safeset = true;
        }
    }

    rcl_interfaces::msg::SetParametersResult 
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters){
        rcl_interfaces::msg::SetParametersResult result;
        for (auto parameter : parameters) {
            const auto & param_type = parameter.get_type();
            const auto & param_name = parameter.get_name();

            if (param_type == ParameterType::PARAMETER_DOUBLE) {
                if (param_name == "alpha") {
                    params->alpha = parameter.as_double();
                    //alpha restricted to be non-negative
                    if (params->alpha <= 0.0) {
                        RCLCPP_WARN(
                            get_logger(), "You've set alpha to be <=0,"
                            " this isn't allowed, alpha needs to be between 0 and 1.");
                        params->alpha = 0.2;
                    }
                } else if (param_name == "delta") {
                    params->delta = parameter.as_double();
                    //alpha restricted to be non-negative
                    if (params->delta <= 0.0) {
                        RCLCPP_WARN(
                            get_logger(), "You've set delta to be <=0,"
                            " this isn't allowed, delta needs to be between 0 and 1.");
                        params->delta = 0.05;
                    }
                } else if (param_name == "b_max") {
                    params->b_max = parameter.as_double();
        
                } else if (param_name == "d") {
                    params->d = parameter.as_double();
                    //alpha restricted to be non-negative
                    if (params->d < 0.0) {
                    RCLCPP_WARN(
                        get_logger(), "You've set the distance parameter to be negative,"
                        " this isn't allowed, so d will be set to 0.05.");
                        params->d = 0.05;
                    }
                } else if (param_name == "r_robot") {
                    params->r_robot = parameter.as_double();
                    //alpha restricted to be non-negative
                    if (params->r_robot < 0.0) {
                    RCLCPP_WARN(
                        get_logger(), "You've set the obstacle radius to be negative,"
                        " this isn't allowed, so r_robot will be set to 0.1.");
                        params->r_robot = 0.1;
                    }
                }
            }
        }

    result.successful = true;
    return result;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PFCBFNode>(rclcpp::NodeOptions());

    rclcpp::Rate rate(50);

    // double begin_time, end_time;
    while (rclcpp::ok()) {
        cout << "defined safeset: " << node->defined_safeset << endl;
        if (node->defined_safeset) {
            node->runControl();
            node->propagateBelief();
        }

    // Call the visualization function
    node->publish_markers();

    rclcpp::spin_some(node);
    rate.sleep();
}

rclcpp::shutdown();
return 0;
}

