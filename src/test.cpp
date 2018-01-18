
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
// #include "visualization_tools/bounding_box.h"
#include "ekf_tracking/tracker.h"

using namespace std;

class HumanTracker
{
	ros::NodeHandle n;
	ros::Subscriber sub;
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

	Tracker tracker;

	public:
	HumanTracker()
		: pc(new pcl::PointCloud<pcl::PointXYZ>)
	{
		sub = n.subscribe<sensor_msgs::PointCloud2>("/human_recognition/positive_position", 1,
				&HumanTracker::humanCallback, this);

		tracker.setThresholdSame(1.0);
		tracker.setThresholdErase(0);
		tracker.setSigma(0.3, 0.3); // P, R
		tracker.setFrameID("/velodyne");
	}

	void humanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
		pcl::fromROSMsg(*msg, *pc);
		tracking();
	}

	void tracking();
};

void HumanTracker::tracking()
{
	tracker.setPosition(pc);

	cout << tracker << endl;
	tracker.pubTrackPoint();
	cout << "pc size : " << pc->points.size() << endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_tracking");
	cout << "ekf tracking" << endl;

	HumanTracker ht;

	ros::spin();

	return 0;
}

