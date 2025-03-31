#ifndef AUTOCROSS_H
#define AUTOCROSS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mmr_base/msg/marker_array.hpp>
#include "mmr_base/msg/race_status.hpp"
#include "utils/Params.hpp"
#include "modules/WayComputer.hpp"
#include "modules/Visualization.hpp"
#include "modules/DelaunayTri.hpp"
#include "utils/Time.hpp"
#include <unistd.h>

// #define DEBUG 1

class AutocrossPlanner
{
	public:
		AutocrossPlanner(const rclcpp::Node::SharedPtr &nh,
						 const rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr &centerLinePub,
						 const rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr &centerLineCompletedPub);

		void raceStatusCallBack(mmr_base::msg::RaceStatus::SharedPtr raceStatus);

		void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);

		void slamConesCallback(mmr_base::msg::Marker::SharedPtr slamCones);
		
		Params *params;
		WayComputer *wayComputer;

	private:
		rclcpp::Node::SharedPtr nh;
		rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr centerLinePub;
		rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr centerLineCompletedPub;

		int currentLap;
		bool idle = false; //when true the node is idle and only publishes centerline_completed in a transient local fashion 
		
		// Params *params;
		// WayComputer *wayComputer;

};

#endif //AUTOCROSS_H