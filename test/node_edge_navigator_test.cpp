#include <gtest/gtest.h>

#include <ros/ros.h>

TEST(TestSuite, test_navigator1)
{
	ros::NodeHandle nh;
	EXPECT_EQ(M_PI, M_PI);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	ros::init(argc, argv, "node_edge_navigator_test");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration(3.0).sleep();

	int r_e_t = RUN_ALL_TESTS();

	spinner.stop();

	ros::shutdown();

	return r_e_t;
}
