#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <wild_thumper/PathFollowingConfig.h>
#include <geometry_msgs/Twist.h>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
static const Scalar& red = CV_RGB(255,0,0);
static const Scalar& orange = CV_RGB(255,165,0);
static const Scalar& green = CV_RGB(0,255,0);
static const Scalar& blue = CV_RGB(0,0,255);
static const Scalar& purple = CV_RGB(128,0,128);

class PathFollower
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	ros::Publisher cmdvel_pub;
	int roi_x;
	int roi_y;
	int binary_threshold;
	int ht_min_points;
	int roi_width, roi_height;
	double x_px_per_cm, y_px_per_cm, y_offset_cm;
	int x_offset_px;
	float lap_x_cm, lap_y_cm;
	float road_distance_cm, lad_cm;
	int img_vsize, img_hsize;
	double axial_dist_cm;
	float speed_m_s;
	Mat transformation_matrix;
	dynamic_reconfigure::Server<wild_thumper::PathFollowingConfig> server;

	public:
	PathFollower() : it(nh)
	{
		ros::NodeHandle pnh("~");
		FileStorage fs(ros::package::getPath("wild_thumper") + "/config/transformation_matrix.yml", FileStorage::READ);
		dynamic_reconfigure::Server<wild_thumper::PathFollowingConfig>::CallbackType f;

		fs["transformation_matrix"] >> transformation_matrix;
		pnh.param("roi_x_init", roi_x, 280);
		pnh.param("roi_width", roi_width, 50);
		pnh.param("roi_height", roi_height, 50);
		pnh.param("x_px_per_cm", x_px_per_cm, 2.778);
		pnh.param("y_px_per_cm", y_px_per_cm, 2.778);
		pnh.param("y_offset_cm", y_offset_cm, 23.5);
		pnh.param("img_vsize", img_vsize, 480);
		pnh.param("img_hsize", img_hsize, 640);
		pnh.param("x_offset_px", x_offset_px, 65);
		pnh.param("axial_dist_cm", axial_dist_cm, 25.4);
		lap_x_cm=0;
		lap_y_cm=0;
		f = boost::bind(&PathFollower::dynreconf_callback, this, _1, _2);
		server.setCallback(f);

		// Subscrive to input video feed and publish output video feed
		image_sub = it.subscribe("/camera/rgb/image_raw", 1, &PathFollower::imageCb, this);
		cmdvel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

		fs.release();

		cv::namedWindow(OPENCV_WINDOW);
	}

	~PathFollower()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	int pathfinder(cv::Mat &frame, float &r, float &phi)
	{
		std::vector<cv::Vec2f> lines;
		cv::warpPerspective(frame, frame, transformation_matrix, frame.size(), INTER_LINEAR);
		Mat roi(frame, Rect(roi_x, roi_y, roi_width, roi_height));

		cv::threshold(roi, roi, binary_threshold, 255, CV_THRESH_BINARY);
		cv::erode(roi, roi, cv::Mat(), cv::Point(-1, -1), 1);
		cv::HoughLines(roi, lines, 1, CV_PI/90, ht_min_points, 0, 0);
		
		if (lines.size() > 0) { // use first result
			r = lines[0][0];
			phi = lines[0][1];

			if (phi > CV_PI/2) { // keep in +-90°
				phi = phi-CV_PI;
				r = -r;
			}

			// Update dynamic ROI
			roi_x = roi_x - roi_width/2 + r;
		}

		return lines.size();
	}

	void calc_lap(float r, float phi) {
		float roi_x_h, lap_x_h, lap_y_h;

		// 1. rotate ROI by phi (in Hilfskoordinatensystem)
		roi_x_h = (cos(-phi)*px_to_cm_x(roi_x) + -sin(-phi)*px_to_cm_y(roi_y));

		// 2. lap_h
		lap_x_h = roi_x_h + r - road_distance_cm;
		// lap_x_h
		lap_y_h = sqrt(lad_cm*lad_cm - lap_x_h*lap_x_h);
		
		// 3. rotate LAP back by phi
		lap_x_cm = (cos(phi)*lap_x_h + -sin(phi)*lap_y_h);
		lap_y_cm = (sin(phi)*lap_x_h + cos(phi)*lap_y_h);
	}

	float pure_pursuit(float r, float phi) {
		calc_lap(r, phi);

		return -atan((2*lap_x_cm*axial_dist_cm)/((float)lad_cm*lad_cm));
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv::Mat cimg;
		float r, phi;
		int htvalid;
		float alpha;

		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		htvalid = pathfinder(cv_ptr->image, r, phi);

		cv::cvtColor(cv_ptr->image, cimg, CV_GRAY2RGB);
		cv::rectangle(cimg, cv::Point(roi_x, roi_y), cv::Point(roi_x+roi_width, roi_y+roi_height), red, 1);

		// grid
		cv::line(cimg, Point(0, cm_to_px_y(50)), Point(img_hsize, cm_to_px_y(50)), blue, 1);
		putText(cimg, " 50cm", Point(img_hsize-60, cm_to_px_y(50)+15), FONT_HERSHEY_PLAIN, 1.0, blue);
		cv::line(cimg, Point(0, cm_to_px_y(60)), Point(img_hsize, cm_to_px_y(60)), blue, 1);
		putText(cimg, " 60cm", Point(img_hsize-60, cm_to_px_y(60)+15), FONT_HERSHEY_PLAIN, 1.0, blue);
		cv::line(cimg, Point(0, cm_to_px_y(70)), Point(img_hsize, cm_to_px_y(70)), blue, 1);
		putText(cimg, " 70cm", Point(img_hsize-60, cm_to_px_y(70)+15), FONT_HERSHEY_PLAIN, 1.0, blue);
		cv::line(cimg, Point(0, cm_to_px_y(80)), Point(img_hsize, cm_to_px_y(80)), blue, 1);
		putText(cimg, " 80cm", Point(img_hsize-60, cm_to_px_y(80)+15), FONT_HERSHEY_PLAIN, 1.0, blue);
		cv::line(cimg, Point(0, cm_to_px_y(100)), Point(img_hsize, cm_to_px_y(100)), blue, 1);
		putText(cimg, "100cm", Point(img_hsize-60, cm_to_px_y(100)+15), FONT_HERSHEY_PLAIN, 1.0, blue);
		// distance
		cv::line(cimg, Point(cm_to_px_x(road_distance_cm), 0), Point(cm_to_px_x(road_distance_cm), img_vsize), purple, 1);

		if (htvalid) {
			double a = cos(phi), b = sin(phi);
			double x0 = a*r + roi_x;
			double y0 = b*r + roi_y;
			Point pt1, pt2;
			pt1.y = 0; 
			pt1.x = r/(cos(phi));
			pt2.y = roi_height;
			pt2.x = (r - pt2.y*sin(phi)) / (cos(phi));
			cv::line(cimg, Point(roi_x, roi_y) + pt1, Point(roi_x, roi_y) + pt2, orange, 2);

			alpha = pure_pursuit(px_to_cm_x(roi_x + r), phi);
			ROS_INFO("alpha=%.2f deg", alpha*180/M_PI);
			if (speed_m_s > 0) {
				set_vel(speed_m_s, alpha);
			}
		}

		cv::circle(cimg, Point(cm_to_px_x(lap_x_cm), cm_to_px_y(lap_y_cm)), 5, green, -1);

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cimg);
		cv::waitKey(3);
	}

	int cm_to_px_x(float cm) {
		return round(cm * x_px_per_cm + img_hsize/2 - x_offset_px);
	}

	double px_to_cm_x(int px) {
		return (px-img_hsize/2+x_offset_px) / x_px_per_cm;
	}

	int cm_to_px_y(float cm) {
		return round((-cm+y_offset_cm) * y_px_per_cm + img_vsize);
	}

	double px_to_cm_y(int px) {
		return (img_vsize-px) / y_px_per_cm + y_offset_cm;
	}

	void dynreconf_callback(wild_thumper::PathFollowingConfig &config, uint32_t level) {
		int px;

		binary_threshold = config.binary_threshold;
		ht_min_points = config.ht_min_points;
		px = cm_to_px_y(config.roi_y);
		if (px > 0 && px < img_vsize - roi_height) roi_y = px;
		else ROS_ERROR("Bad roi_y position %.2fcm", roi_y);
		road_distance_cm = config.road_distance_cm;
		lad_cm = config.lad_cm;	
		speed_m_s = config.speed_m_s;
	}

	void set_vel(float speed, float alpha) {
		geometry_msgs::Twist msg;

		msg.linear.x = speed;
		msg.angular.z = alpha;
		
		cmdvel_pub.publish(msg);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_following");

	PathFollower ic;
	ros::spin();
	return 0;
}
