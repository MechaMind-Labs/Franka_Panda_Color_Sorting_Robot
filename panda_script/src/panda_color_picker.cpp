#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>
#include <map>

class PandaColorPicker : public rclcpp::Node
{
public:
    PandaColorPicker() : Node("panda_color_picker")
    {
        // ---------------- Parameters ----------------
        this->declare_parameter<std::string>("pick_order", "RGB");
        this->get_parameter("pick_order", pick_order_);
        RCLCPP_INFO(get_logger(), "Pick order set to: %s", pick_order_.c_str());

        // ---------------- Subscriber ----------------
        color_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/color_coordinates", 10,
            std::bind(&PandaColorPicker::colorCallback, this, std::placeholders::_1));

        // ---------------- MoveIt ----------------
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "hand");

        // Set planner params
        arm_->setPlanningTime(5.0);
        arm_->setMaxVelocityScalingFactor(0.5);
        arm_->setMaxAccelerationScalingFactor(0.5);

        // Predefined drop locations (example)
        drop_positions_["R"] = {0.5, 0.2, 0.1};
        drop_positions_["G"] = {0.5, 0.0, 0.1};
        drop_positions_["B"] = {0.5, -0.2, 0.1};

        RCLCPP_INFO(get_logger(), "PandaColorPicker Node Started");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_, gripper_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_sub_;
    std::mutex color_mutex_;

    struct ColorPoint { std::string id; double x, y, z; };
    std::map<std::string, ColorPoint> detected_colors_;
    std::string pick_order_;
    std::map<std::string, std::vector<double>> drop_positions_;

    void colorCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(color_mutex_);
        // Parse message: format "R,0.123,0.456,0.789"
        std::stringstream ss(msg->data);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) tokens.push_back(token);

        if (tokens.size() != 4) return;

        ColorPoint cp;
        cp.id = tokens[0];
        cp.x = std::stod(tokens[1]);
        cp.y = std::stod(tokens[2]);
        cp.z = std::stod(tokens[3]);

        detected_colors_[cp.id] = cp;

        RCLCPP_INFO(get_logger(), "Detected color %s at %.3f, %.3f, %.3f", 
                    cp.id.c_str(), cp.x, cp.y, cp.z);

        // Try pick in order
        pickColorsInOrder();
    }

    void pickColorsInOrder()
    {
        for (char color_char : pick_order_)
        {
            std::string color(1, color_char);
            if (detected_colors_.find(color) != detected_colors_.end())
            {
                ColorPoint cp = detected_colors_[color];

                RCLCPP_INFO(get_logger(), "Picking color %s", color.c_str());

                // ---------------- Move to above color ----------------
                geometry_msgs::msg::Pose target_pose;
                target_pose.position.x = cp.x;
                target_pose.position.y = cp.y;
                target_pose.position.z = cp.z + 0.1; // approach from above

                // Orientation: pointing down
                target_pose.orientation.x = 0.0;
                target_pose.orientation.y = 1.0;
                target_pose.orientation.z = 0.0;
                target_pose.orientation.w = 0.0;

                arm_->setPoseTarget(target_pose);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success) arm_->move();
                else
                {
                    RCLCPP_WARN(get_logger(), "Failed to plan to color %s", color.c_str());
                    continue;
                }

                // ---------------- Lower to pick ----------------
                target_pose.position.z = cp.z;
                arm_->setPoseTarget(target_pose);
                arm_->move();

                // ---------------- Close gripper ----------------
                gripper_->setJointValueTarget({0.8}); // adjust as per gripper
                gripper_->move();

                // ---------------- Lift ----------------
                target_pose.position.z = cp.z + 0.15;
                arm_->setPoseTarget(target_pose);
                arm_->move();

                // ---------------- Move to drop location ----------------
                std::vector<double> drop = drop_positions_[color];
                target_pose.position.x = drop[0];
                target_pose.position.y = drop[1];
                target_pose.position.z = drop[2] + 0.15;
                arm_->setPoseTarget(target_pose);
                arm_->move();

                // ---------------- Lower to drop ----------------
                target_pose.position.z = drop[2];
                arm_->setPoseTarget(target_pose);
                arm_->move();

                // ---------------- Open gripper ----------------
                gripper_->setJointValueTarget({0.0});
                gripper_->move();

                // ---------------- Retract ----------------
                target_pose.position.z = drop[2] + 0.15;
                arm_->setPoseTarget(target_pose);
                arm_->move();

                // Remove the color from detected_colors_ so it is not picked again
                detected_colors_.erase(color);
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PandaColorPicker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
