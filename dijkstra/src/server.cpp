#include <thread>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dijkstra/action/action.hpp"
#include "dijkstra/map.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using dijkstra::action::Action;
using GoalHandleAction = rclcpp_action::ServerGoalHandle<Action>;

rclcpp::Node::SharedPtr node;
rclcpp_action::Server<Action>::SharedPtr server;

rclcpp_action::GoalResponse handle_goal(
	const rclcpp_action::GoalUUID&,
	std::shared_ptr<const Action::Goal> goal
) {
	std::ostringstream stream;
	for (const auto& [first_node, second_node, distance]: goal->map) {
		stream << "First node: " << first_node << ", Second node: " << second_node << ", Relative distance: " << distance << "\n";
	}
	stream << "Source: " << goal->source;
	RCLCPP_INFO(node->get_logger(), "Goal accepeted:\n%s", stream.str().c_str());
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAction>) {
	RCLCPP_INFO(node->get_logger(), "Received request to cancel goal");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(const std::shared_ptr<GoalHandleAction> goal_handle) {
	rclcpp::Rate rate(1);
	auto solver = std::make_shared<Solver>(goal_handle->get_goal());
	while (!solver->is_finished() && rclcpp::ok()) {
		if (goal_handle->is_canceling()) {
			goal_handle->canceled(solver->get_current_result());
			RCLCPP_INFO(node->get_logger(), "Goal canceled");
			return;
		}
		auto feedback = solver->continue_solving();
		while (feedback == nullptr && !solver->is_finished()) {
			feedback = solver->continue_solving();
		}
		if (!solver->is_finished()) {
			goal_handle->publish_feedback(feedback);
			rate.sleep();
		}
	}

	if (rclcpp::ok()) {
		auto result = solver->get_current_result();
		goal_handle->succeed(result);
		RCLCPP_INFO(node->get_logger(), "Goal succeeded");
	}
}

void handle_accept(const std::shared_ptr<GoalHandleAction> goal_handle) {
	std::thread(execute, goal_handle).detach();
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	node = rclcpp::Node::make_shared("server");
	server = rclcpp_action::create_server<Action>(
		node,
		"action",
		handle_goal,
		handle_cancel,
		handle_accept
	);
	
	RCLCPP_INFO(node->get_logger(), "Server ready");
	rclcpp::spin(node);
	rclcpp::shutdown();
}
