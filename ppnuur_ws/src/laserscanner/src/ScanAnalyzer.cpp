#define FIXED_MODE 1

#include "ros/ros.h"
#include <string>
#include <iostream>
#include "ScanAnalyzer.h"
#include <list>
#include <limits>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
//Includes Map to Rviz
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace cv;
int mode = 0;
/*
 Konstruktor der Scan_analyser Klasse
 Es wird eine Roboterposition, verwendete Konstanten und die Publisher/Subscriber initialisiert
 */
ScanAnalyzer::ScanAnalyzer(ros::NodeHandle n) {
	n_ = n;
	angle_increment = 0.0;
	angle_min = 0;
	min_dist = 0.01;
	max_dist = 4.00;
	depth_jump = 0.01;
	merge_radius = 0.03;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",
			1);
	subScan = n.subscribe("scan", 1, &ScanAnalyzer::scanner_callback, this);
	c=1;
	s=0;
	tx=0;
	ty=0;
}

void ScanAnalyzer::draw_icp(const vector<Point2d>& cloud,
		const vector<Point2d>& last_cloud, cv::Mat R, cv::Mat T) {
	Mat img(1000, 1000, CV_8UC3, Scalar(255, 255, 255));
	Point2d center(img.cols / 2, img.rows / 2);
	double resolution = 500;
	for (const auto& p : cloud) {
		Point2d v = p * resolution + center;
		circle(img, v, 2, Scalar(0, 0, 255), -1);
	}
	for (const auto& p : last_cloud) {
		Point2d v = p * resolution + center;
		circle(img, v, 2, Scalar(255, 0, 0), -1);
		Point2d t;
		t.x = R.at<double>(0, 0) * p.x + R.at<double>(0, 1) * p.y
				+ T.at<double>(0, 0);
		t.y = R.at<double>(1, 0) * p.x + R.at<double>(1, 1) * p.y
				+ T.at<double>(1, 0);
		Point2d v2 = t * resolution + center;
		circle(img, v2, 2, Scalar(0, 255, 0), -1);
	}
	imshow("ICP", img);
	waitKey(10);
}

vector<pair<Point2d, Point2d> > ScanAnalyzer::assignemnt(
		const vector<Point2d>& cloud, const vector<Point2d>& l) {
	vector<pair<Point2d, Point2d> > assignments;
	return assignments;
}

/*
 Main
 */
cv::Mat ScanAnalyzer::liDarOdom(vector<float> scan) {
	Mat proj= Mat::eye(Size(3, 3), CV_64F);
	static vector<Point2d> last_cloud;
	vector<Point2d> cloud;
	for (int i = 0; i < scan.size(); i++) {
		double f = scan[i];
		if (isnan(f) || isinf(f))
			continue;
		double a = i * angle_increment - angle_min;
		cloud.push_back(Point2d(f * cos(a), f * sin(a)));
	}
	if (last_cloud.size() > 2 && cloud.size() > 2) {
		cv::Mat R = (Mat_<double>(2, 2) << 1, 0, 0, 1);
		cv::Mat T = (Mat_<double>(2, 1) << 0, 0);
		vector<Point2d> l;
		for (int i = 0; i < 1; i++) {
			draw_icp(cloud, last_cloud, R, T);
			l.clear();
			/*
			 * todo Implementieren Sie den ICP Algorithmus
			 * Ordnen Sie Punktpaare zu
			 * Reduzieren Sie die Paare um ihren Schwerpunkt
			 * Berechnen Sie R und t
			 * Passen sie das globale R und t an
			 * Iterieren Sie bis ein Abbruch kriterium erreicht ist
			 */
		}

		Mat dst_roi = proj(Rect(0, 0, 2, 2));
		R.copyTo(dst_roi);
		dst_roi = proj(Rect(0, 2, 1, 2));
		T.copyTo(dst_roi);
	}
	last_cloud = cloud;

	return proj;

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ScanAnalyzer");
	ros::NodeHandle n;
	ScanAnalyzer m = ScanAnalyzer(n);
	ros::spin();
	return 0;
}

void ScanAnalyzer::calculate_transformation(
		std::vector<ScanAnalyzer::Pole> scan1,
		std::vector<ScanAnalyzer::Pole> scan2) {
	cv::Point2d cl, cr;
	cv::Mat l(scan1.size() * 2, 1, CV_64FC1, cv::Scalar(0));
	cv::Mat x(4, 1, CV_64FC1, cv::Scalar(0));
	cv::Mat A(scan1.size() * 2, 4, CV_64FC1, cv::Scalar(0));
	cv::Mat At, A_1;
	cv::transpose(A, At);
	cv::Mat AtA = At * A;
	cv::invert(AtA, A_1, cv::DECOMP_SVD);
	cv::Mat tmpx = At * l;
	x = A_1 * tmpx;
	std::cout << x << std::endl;
	c = x.at<double>(0, 0);
	s = x.at<double>(1, 0);
	tx = x.at<double>(2, 0);
	ty = x.at<double>(3, 0);

}

std::vector<std::pair<int, int> > ScanAnalyzer::pick_assignements() {
	char input[100];
	int a, b;
	std::vector<std::pair<int, int> > assignements;
	while (strcmp(input, "q") != 0) {
		std::cout << "Pick Pole scan1 or continue(q) (" << scan1.size()
				<< " Poles)" << std::endl;
		std::cin >> input;
		if (strcmp(input, "q") == 0)
			break;

		a = atoi(input);
		std::cout << "Pick Pole scan2 or continue(q) (" << scan2.size()
				<< " Poles)" << std::endl;
		std::cin >> input;
		if (strcmp(input, "q") == 0)
			break;

		b = atoi(input);
		assignements.push_back(std::pair<int, int>(a, b));
	}
	return assignements;
}

void ScanAnalyzer::publish_markers() {
	for (int i = 0; i < map.size(); i++) {
		ROS_INFO("Pole %d is at: ( %f, %f)!", i + 1, map[i].x, map[i].y);
		visualization_msgs::Marker marker;
		marker.header.frame_id = "laser";
		marker.header.stamp = ros::Time::now();
		marker.ns = "mapPoles";
		marker.id = i;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = map[i].x;
		marker.pose.position.y = map[i].y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.01;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker_pub.publish(marker);
	}
}

/*
 Scanner Callback
 Es wird für jeden erkannten Scan die vollständige Routine von:
 1. Ableitung bilden
 2. Stangen detektieren
 3. mapUpdate berechen
 durchlaufen
 */

void ScanAnalyzer::scanner_callback(const sensor_msgs::LaserScan &scan) {
	angle_increment = scan.angle_increment;
	angle_min = scan.angle_min;
	actScan = scan.ranges;

	std::cout << "Type 0 for show actual scan + transformed scan 1" << std::endl
			<< "Type 1 or 2 to save actual scan as scan 1 or 2" << std::endl
			<< "Type 3 for calculation of transformation" << std::endl
			<< "Type 4 for switch to LiDAR odometry" << std::endl;
#ifndef FIXED_MODE
	if (mode != 4) {
		char input[100];
		std::cin >> input;
		mode = atoi(input);
	}
#else
	mode = FIXED_MODE;
	std::cout << "Type " << mode << std::endl;
#endif

	std::vector<float> der = compute_derivative(actScan);
	switch (mode) {
	case 1:
		scan1 = get_update(actScan, der);
		map = scan1;
		break;
	case 2:
		scan2 = get_update(actScan, der);
		map = scan2;
		break;
	case 3: {
		std::vector<std::pair<int, int>> assignments = pick_assignements();
		std::vector<ScanAnalyzer::Pole> pole1;
		std::vector<ScanAnalyzer::Pole> pole2;
		for (auto p : assignments) {
			pole1.push_back(scan1[p.first]);
			pole2.push_back(scan2[p.second]);
		}
		calculate_transformation(pole1, pole2);
		map = scan2;
		break;
	}
	case 4: {
		Mat p=liDarOdom(scan.ranges);
		cout<<p<<endl;
		char c = waitKey(10);
		if (isdigit(c)) {
			int n = atoi(&c);
			if (n < 5)
				mode = 5;
		}
		break;
	}
	default:
		map = get_update(actScan, der);
		break;
	}
	std::vector<ScanAnalyzer::Pole> mapUpdate;
	for (int i = 0; i < scan1.size(); i++) {
		double xNew = c * scan1[i].x - s * scan1[i].y + tx;
		double yNew = s * scan1[i].x + c * scan1[i].y + ty;
		std::cout << "New Pole " << i << ":" << xNew << "," << yNew
				<< std::endl;
		mapUpdate.push_back(Pole(xNew, yNew, scan1[i].r));
	}
	merge_map(mapUpdate);
	publish_markers();
}

/*
 Mergen der Map:
 Der aktuell Scan wird mit der Karte verglichen Säulen an einer neuen position werden zur Karte hinzugefügt.
 Dabei werden neue Säulen durch Ihren Abstand zu alten Säulen erkannt.
 Eine Säule die sich in einem Radius unter dem mindest Abstand "merg_radius" befindet,
 wird als die selbe Säule definiert und nicht erneut hinzugefügt.
 */
int ScanAnalyzer::merge_map(std::vector<ScanAnalyzer::Pole> mapUpdate) {
	float dist = 0.0, xi, yi, xj, yj, dx, dy;
	for (int i = 0; i < mapUpdate.size(); i++) {
		bool add = true;
		xi = mapUpdate[i].x;
		yi = mapUpdate[i].y;
		for (int j = 0; j < map.size(); j++) {
			xj = map[j].x;
			yj = map[j].y;
			dx = std::abs(xi - xj);
			dy = std::abs(yi - yj);
			dist = sqrt(dx * dx + dy * dy);
			if (dist < merge_radius) {
				add = false;
			}
		} //for map
		if (add)
			map.push_back(mapUpdate[i]);
	} //for mapUpdate
	return 0;
}

/*
 Allgemein werden Messpunkt entweder einer Säule zugeordnet oder nicht, dazu wird der Beginn und das Ende einer Säule mit Hilfe der Ableitung festgestellt.
 Der Durchschnitt der Messpunkte auf einer Säule wird zur Berechnung Ihrer Koordinaten verwendet.
 1. Eine Säule beginnt bei einem negativen Sprung in der Ableitung.
 2. Addiere die Distanz und den Strahlindex, inkrementiere Anzahl der Strahlen auf der Stange
 3. Detektiere positiven Sprung als Stangenende
 4. Berechne die Mittelwerte für die Stangenposition compute_cartesian.
 5. Füge die Stange dem mapUpdate hinzu.
 */
std::vector<ScanAnalyzer::Pole> ScanAnalyzer::get_update(
		const std::vector<float> scan, const std::vector<float> der) {
	bool on_cylinder = false;
	std::vector<ScanAnalyzer::Pole> mapUpdate;
	/*
	 * to Do Laboraufgabe:
	 * 1. Skippen Sie ungültige Werte (min_dist,nan,inf).
	 * 2. Identifizierung des  Stangenanfang, die Nachfolgenden Punkte liegen auf der Stange.
	 * * 3. Detektion des Stangensende und Berechnung des Durchschnitt aller Messwerte auf der Stange am Stangenende
	 * 	5. Berechnet die Koordinaten einer neuen Stange im lokalen Scannerkoordinatensystem
	 * 		Nutzen Sie compute_cartesian
	 * 			Nutzen Sie  Start- ,Endindex, Winkelinkrement und Minimalwinkel um:
	 * 				den Radius der Stange zu berechnen
	 * 				die Entfernung des Mittelpunkt der Stange zum Scanner
	 * 				den Winkel des Strahles der den Mittelpunkt trifft
	 * 	6. Verfollständigung des mapUpdates
	 */

	return mapUpdate;
}

/*
 Bestimmen der localen 2D-kartesischen Koordinaten aus Strecke und Winkel des Scans.
 */
ScanAnalyzer::Pole ScanAnalyzer::compute_cartesian(float radius, float distance,
		float angle) {
	float x = 0, y = 0, r = 0, depth = 0, theta = 0;
	x = distance * cos(angle);
	y = distance * sin(angle);
	ScanAnalyzer::Pole newPole = { x, y, radius };
	return newPole;
}

/*
 compute_derivaion berechnet die Ableitung für einen Scan
 Die numerischen Ableitung  wird aus der 2er-Nachbarschaft gebildet.
 Da mit dem jeweils linken und rechten Nachbarn der aktuellen Position
 die numerische Ableitung gebildet wird, muss die erste
 und letzte Stelle des Arrays mit 0.0 beschrieben werden,
 um eine konstante Array-Länge zu gewährleisten.
 */
std::vector<float> ScanAnalyzer::compute_derivative(
		const std::vector<float> actScan) {
	std::vector<float> der;
	//first element zero
	der.push_back(0.0);

	//last element zero
	der.push_back(0.0);
	return der;
}
ScanAnalyzer::~ScanAnalyzer() {
}

