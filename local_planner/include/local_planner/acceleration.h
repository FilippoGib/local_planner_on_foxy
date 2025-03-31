#ifndef ACCELERATION_H
#define ACCELERATION_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mmr_base/msg/marker_array.hpp>
#include "mmr_base/msg/race_status.hpp"
#include <unistd.h>

class AccelerationPlanner
{
	public:
		AccelerationPlanner(rclcpp::Node::SharedPtr nh,
							rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr bordersPub,
							rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr centerLinePub);

		void loadParameters();

		void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);

		void slamConesCallback(mmr_base::msg::Marker::SharedPtr slamCones);

		std::vector<geometry_msgs::msg::Point> generateDiscretizedLine(const geometry_msgs::msg::Point &pointStart,
										           const geometry_msgs::msg::Point &pointEnd,
												   const double &horizon);

		std::vector<geometry_msgs::msg::Point> generateBorder(const std::vector<geometry_msgs::msg::Point> &slamCones,
										  const bool &borderType,
										  const double &horizon);

		void generateCenterLine(const std::vector<geometry_msgs::msg::Point> &borderL,
								const std::vector<geometry_msgs::msg::Point> &borderR);

	private:
		rclcpp::Node::SharedPtr nh;
		rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr bordersPub;
		rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr centerLinePub;

		mmr_base::msg::MarkerArray borders;
		geometry_msgs::msg::Point odometry;

		double param_lineWidth;
};

#endif //ACCELERATION_H