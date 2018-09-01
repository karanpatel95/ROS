#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure_ex/exampleConfig.h>

#include <ros/console.h>

void callback(dynamic_reconfigure_ex::exampleConfig &config, uint32_t level) {
	
	std::string bool_check = config.bool_param ? "True" : "False";

	ROS_INFO_STREAM("Reconfigure Request : " << 
		config.int_param <<
		config.double_param << 
		config.str_param <<
		bool_check <<
		config.size);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "dynamic_reconfigure_example");

	ros::NodeHandle nh;

	dynamic_reconfigure::Server<dynamic_reconfigure_ex::exampleConfig> server;
	dynamic_reconfigure::Server<dynamic_reconfigure_ex::exampleConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ROS_INFO("Spinning node");
	ros::spin();
	return 0;
}