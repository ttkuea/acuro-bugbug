

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <vector>
#include <cmath>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"


#include <cpprest/http_client.h>
#include <cpprest/filestream.h>
#include <locale>         // std::wstring_convert
#include <codecvt>        // std::codecvt_utf8

// #include "Mode/ModeClass.h"

#define MODE_NEUTRAL 0
#define MODE_DEFAULT 1
#define MODE_CALIBRATE 2
#define MODE_LINE 3

#define CALIBRATE_NUMBER 200

using namespace std;
using namespace std::chrono;
using namespace cv;

using namespace utility;
using namespace web;
using namespace web::http;
using namespace web::http::client;
using namespace concurrency::streams;

#define Create_Comport "COM3"
#define M_PI 3.14159265358979323846 
bool isRecord = false;
unsigned int modeType = MODE_LINE;

// http client
http_client http_c(U("http://localhost:5000"));

// string converter
typedef std::codecvt_utf8<wchar_t> ccvt;
std::wstring_convert<ccvt> wstr_convert;

//Mode *mode = nullptr;

double angle2rad(double angle)
{
	return angle * M_PI / 180.0;
}

vector<double> godFunction(long long diff)
{
	// Tune
	const double R = 300;   // mm
	const double T = 15000; // millisec
	// Tune

	vector<double> output;

	double deltaT = diff;
	double theta = (deltaT / T) * 360.0;

	double alpha = (180 - theta) / 2;
	double displacement = 2 * R * sin(angle2rad(theta / 2));

	output.push_back(displacement * sin(angle2rad(90 - alpha))); // delta x
	output.push_back(displacement * cos(angle2rad(90 - alpha))); // delta y
	output.push_back(theta);                                     // delta angle
	return output;
}

void deltaPos(double vl, double vr, long long dt_ns, double theta, double* dx, double* dy, double* dTheta)
{
	double r = 312.5;
	double v = (vl + vr) / 2;
	double w = (vl - vr) / r;

	double dt_s = dt_ns * 1e-3;

	*dx = v * dt_s * sin(w * dt_s + angle2rad(theta));
	*dy = v * dt_s * cos(w * dt_s + angle2rad(theta));
	*dTheta = w * dt_s * 180.0 / M_PI;
}

void deltaPos3(double vl, double vr, long long dt_ns, double theta, double* dx, double* dy, double* dTheta)
{
	double r = 312.5;
	double w = (vl - vr) / r;

	double dt_s = dt_ns * 1e-3;

	*dx = sin(w * dt_s + angle2rad(theta));
	*dy = cos(w * dt_s + angle2rad(theta));
	*dTheta = w * dt_s * 180.0 / M_PI;
}

void deltaPos2(int dist_mm, int angle_degree, double theta, double* dx, double* dy)
{
	double dRad = angle2rad((double)angle_degree);
	*dx = dist_mm * sin(dRad + angle2rad(theta));
	*dy = dist_mm * cos(dRad + angle2rad(theta));
}

double distance(pair<double,double> p1, pair<double, double> p2)
{
	return sqrt(pow(p2.first - p1.first, 2) + pow(p2.second - p1.second, 2));
}

double perimeter(vector<pair<double, double>> pos, double posX0, double posY0)
{
	double sum = 0;
	for (int i = 0; i < pos.size() - 1; ++i) {
		sum += distance(pos[i], pos[i + 1]);
	}
	sum += distance(pos[pos.size() - 1], make_pair(posX0, posY0));
	return sum;
}



string get_stringData() {
	// Create http_client to send the request.
	http_response res = http_c.request(methods::GET).get();
	std::wstring res_wstr = res.extract_string().get();
	std::string res_str = wstr_convert.to_bytes(res_wstr);

	return res_str;
}

typedef struct RobotState {
	double position[3] = { 0.0f, 0.0f, 0.0f };
	double orientation = 0.0f;
} RobotState;

void get_RobotState(RobotState* robotState, string stringData) {
	sscanf(stringData.c_str(), "%lf,%lf,%lf,%lf", &robotState->position[0], 
												  &robotState->position[1], 
											      &robotState->position[2], 
											      &robotState->orientation);
}


int main()
{
	cout << "Main : Started" << endl;

	auto stringData = get_stringData();
	RobotState robotState;
	get_RobotState(&robotState, stringData);
	std::cout << robotState.position[0] << " " 
			  << robotState.position[1] << " " 
			  << robotState.position[2] << " " 
			  << robotState.orientation << std::endl;
	/*
	int width = 400, height = 400;
	IplImage* img = cvCreateImage(cvSize(width, height), 8, 3);
	CreateData robotData;
	RobotConnector robot;

	ofstream record;
	record.open("../data/robot.txt");

	cout << "Robot : Connecting..." << endl;
	if (!robot.Connect(Create_Comport))
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}
	cout << "Robot : Connected!" << endl;

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");

	int velL = 0;
	int velR = 0;

	//state kub
	int state = 0;
	int accangle = 0;
	double posX0 = 0;
	double posY0 = 0;
	double theta0 = 0;
	int counter = 0;

	robot.DriveDirect(0, 0);

	long long stamp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	double posX = 0, posY = 0;
	double theta = 0;
	double angle = 90;

	cout << posX << " " << posY << " " << theta << endl;

	vector<pair<double, double>> pos_stack;
	pair<double, double> last_pos_bump;
	long long last_bump_time = stamp;
	while (true)
	{
		if (!robot.ReadData(robotData))
		{
			cout << "ReadData Fail" << endl;
		}
		
		cout << CV_VERSION;
	}

	cout << perimeter(pos_stack, posX0, posY0) << " " << perimeter(pos_stack, posX0, posY0) * 0.61804009 << endl;

	robot.DriveDirect(0, 0);
	cvWaitKey(50);
	cout << "Robot : Disconnecting..." << endl;
	robot.Disconnect();
	cout << "Robot : Disconnected !" << endl;
	cvWaitKey(0);
	*/
	return 0;
}
