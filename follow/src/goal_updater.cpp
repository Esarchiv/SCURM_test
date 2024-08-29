#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

class GoalUpdater : public BT::SyncActionNode
{
public:
    GoalUpdater(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<geometry_msgs::msg::Pose>("input_goal"), BT::OutputPort<geometry_msgs::msg::Pose>("updated_goal") };
    }

    BT::NodeStatus tick() override
    {
        geometry_msgs::msg::Pose input_goal;
        if (!getInput("input_goal", input_goal))
        {
            throw BT::RuntimeError("missing required input [input_goal]");
        }

        geometry_msgs::msg::Pose updated_goal = updateGoal(input_goal);

        setOutput("updated_goal", updated_goal);
        return BT::NodeStatus::SUCCESS;
    }

private:
    geometry_msgs::msg::Pose updateGoal(const geometry_msgs::msg::Pose& input_goal)
    {
        geometry_msgs::msg::Pose updated_goal = input_goal;
        updated_goal.position.x += 1.0;
        return updated_goal;
    }
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<GoalUpdater>("GoalUpdater");
}
