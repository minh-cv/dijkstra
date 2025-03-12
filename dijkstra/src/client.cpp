#include <sstream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dijkstra/action/action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using dijkstra::action::Action;
using dijkstra::msg::NodeInfo;
using dijkstra::msg::NodesPair;
using GoalHandleAction = rclcpp_action::ClientGoalHandle<Action>;

rclcpp::Node::SharedPtr node;
rclcpp_action::Client<Action>::SharedPtr client;

std::atomic<bool> end_program{false};

void goal_response_callack(const GoalHandleAction::SharedPtr& goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
	} else {
		RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
	}
}

void feedback_callback(GoalHandleAction::SharedPtr, const std::shared_ptr<const Action::Feedback> feedback) {
    std::ostringstream stream;
	stream << "\tNode name: " << feedback->current_node.node_name
			<< "\n\tAbsolute distance: " << feedback->current_node.distance
			<< "\n\tPath: " << feedback->current_node.path;
	RCLCPP_INFO(node->get_logger(), "Current feedback:\n%s", stream.str().c_str());
}

void result_callback(const GoalHandleAction::WrappedResult& result) {
	switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        return;
    }

	std::ostringstream stream;
	for (const NodeInfo& node: result.result->result) {
		stream << "\n\tNode name: " << node.node_name
				<< ", Distance: " << node.distance
				<< ", Path: " << node.path;
	}
	RCLCPP_INFO(node->get_logger(), "Result received:%s", stream.str().c_str());

	rclcpp::shutdown();
}

auto send_goal(Action::Goal goal) {
	rclcpp_action::Client<Action>::SendGoalOptions send_goal_options;
	send_goal_options.feedback_callback = feedback_callback;
	send_goal_options.goal_response_callback = goal_response_callack;
	send_goal_options.result_callback = result_callback;
	RCLCPP_INFO(node->get_logger(), "Sending goal");
	return client->async_send_goal(goal, send_goal_options);
}

Action::Goal parse_goal(const std::string& file_path) {
	std::ifstream stream{file_path};
	Action::Goal goal;
	std::string first_node, second_node;
	unsigned int distance;
	
	stream >> goal.source;
	while (stream >> first_node >> second_node >> distance) {
		NodesPair p;
		p.first_node_name = first_node;
		p.second_node_name = second_node;
		p.distance = distance;
		goal.map.push_back(p);
	}
	
	return goal;
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	
	if (argc != 2) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: client file_path");
		return 1;
	}

	node = rclcpp::Node::make_shared("client");
	client = rclcpp_action::create_client<Action>(node, "action");
	RCLCPP_INFO(node->get_logger(), "Client ready");

	while (!client->wait_for_action_server(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		else {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
		}
	}
	
	send_goal(parse_goal(argv[1]));
	rclcpp::spin(node);
}
