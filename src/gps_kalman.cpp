#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include "gps.h"
#include <libgpsmm.h>

using namespace gps_common;

double longitude = 0.0;
double latitude = 0.0;

void gps_callback(const GPSFixConstPtr& coord) {
	longitude = coord->longitude;
	latitude = coord->latitude;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kalman_gps");
	ros::NodeHandle n;

	ros::Subscriber gps_sub = n.subscribe("extended_fix", 10, gps_callback);
	ros::Publisher kf_gps_pub = n.advertise<GPSFix>("kf_gps", 10);

	KalmanFilter f = alloc_filter_velocity2d(0.0001); //0.0001 is the noise, might need to change

	ros::Rate r(1);

	double lon, lat;

	while (ros::ok()) {
		ROS_INFO("running ..");
		update_velocity2d(f, latitude, longitude, 1);
		get_lat_long(f, &lat, &lon);

		GPSFix fix;
		fix.longitude = lon;
		fix.latitude = lat;
		kf_gps_pub.publish(fix);
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}