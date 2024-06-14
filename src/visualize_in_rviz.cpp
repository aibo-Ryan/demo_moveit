#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("visualize_in_rviz", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executors;
    executors.add_node(node);
    std::thread([&executors]()
                { executors.spin(); })
        .detach();
    auto const logger = rclcpp::get_logger("visualize_in_rviz");
    RCLCPP_INFO(logger, "start ");

    auto move_group = moveit::planning_interface::MoveGroupInterface(node, "panda_arm");
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools(node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group.getRobotModel());
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    auto const draw_title = [&moveit_visual_tools](auto text)
    {
        auto const text_pose = []
        {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0; // Place text 1m above the base link
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XLARGE);
    };

    auto const prompt = [&moveit_visual_tools](auto text)
    {
        moveit_visual_tools.prompt(text);
    };
    
    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
         jmg = move_group.getRobotModel()->getJointModelGroup(
             "panda_arm")](auto const trajectory)
    {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };

    // Set a target Pose
    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.28;
        msg.position.y = -0.2;
        msg.position.z = 0.5;
        return msg;
    }();
    move_group.setPoseTarget(target_pose);

    // Create a plan to that target pose
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title("Planning");
    moveit_visual_tools.trigger();
    auto const [success, plan] = [&move_group]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group.execute(plan);
    }
    else
    {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    rclcpp::shutdown();

    return 0;
}