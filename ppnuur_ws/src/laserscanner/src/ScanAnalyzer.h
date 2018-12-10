#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>

class ScanAnalyzer {

public:
	struct Pole {
		Pole(double x,  double y, double r) : x(x), y(y), r(r) {};
		float x, y, r;
	};

	double c,s,tx,ty;

	// Attributes
	std::vector<Pole>   scan1, scan2;
	std::vector<float>  actScan;
	std::vector<Pole>   map;
    
    
	ros::NodeHandle     node;
    
    ros::Publisher  marker_pub;
    ros::Subscriber subScan;
    
    float angle_increment;
    float angle_min;
	float min_dist;
	float max_dist;
	float depth_jump;
	float merge_radius;
    
	// Methods
	ScanAnalyzer(ros::NodeHandle n);
	~ScanAnalyzer();

	int                 merge_map(std::vector<Pole> mapUpdate);
	Pole                compute_cartesian(float mean_ray, float mean_depth, float rays);
	void                scanner_callback(const sensor_msgs::LaserScan &scan);
	std::vector<Pole>   get_update(std::vector<float> scan,std::vector<float> der);
	std::vector<float>  compute_derivative(const std::vector<float> actScan);
	void                calculate_transformation(std::vector<ScanAnalyzer::Pole> scan1,
                                                 std::vector<ScanAnalyzer::Pole> scan2);
	void                publish_markers() ;
	cv::Mat             liDarOdom(std::vector<float> scan);
    
    std::vector<std::pair<int, int>>   pick_assignements();

private:
	void draw_icp(const std::vector<cv::Point2d>& cloud,
                  const std::vector<cv::Point2d>& last_cloud,
                  cv::Mat R,
                  cv::Mat T);
    
	std::vector<std::pair<cv::Point2d, cv::Point2d>> assignemnt(
                const std::vector<cv::Point2d>& cloud,
                const std::vector<cv::Point2d>& l);
};
