// test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Kinect.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <Windows.h>
#include <iostream>
#include <time.h> 
#include <algorithm>
#include <fstream>

#include "k2p.h"
#include "dc.h"

#pragma comment(lib,"Winmm.lib")
using namespace std;


//safe way of deleting a COM object
template<typename T>
void SafeRelease(T& ptr) { if (ptr) { ptr->Release(); ptr = nullptr; } }
   // Kinect depth data source
IKinectSensor* sensor = nullptr;
IBodyFrameReader* bodyReader;
BOOL Point_trigger = false;
//BOOL Click_trigger = false;
BOOL calibration = false;
BOOL calibrationPoint[4] = { false, false, false, false };
//BOOL calibrationPoint2 = false;
//BOOL calibrationPoint3 = false;
//BOOL calibrationPoint4 = false;
//double current_x, current_y;
time_t t0;
double A[4][3];
Kinect2Projector k2p;
double screen_x, screen_y;
//int pointNumber = 0;
double dx, dy;
struct calibPoint
{
	BOOL ClickTriger = false;
	double A[4][3];
	double current_x;
	double current_y;
	bool calibrationPoint[4] = { false, false, false, false };
	int pointNumber = 0;
	int handTrackerChecker = 0;
	double handTrackerPointsX[4][11];
	double handTrackerPointsY[4][11];
	double handTrackerPointsZ[4][11];
};



void point_calibration(double correctedDistance, double distanceToScreen, int numberOfCalibrationPoints, ColorSpacePoint colorpoint, CameraSpacePoint point, calibPoint& calibPoint)
{
	
	if (correctedDistance < distanceToScreen && calibPoint.ClickTriger == false && (calibPoint.pointNumber == 0 || (colorpoint.X - calibPoint.current_x > 500 && (calibPoint.pointNumber == 1 || calibPoint.pointNumber == 3)) || (colorpoint.Y - calibPoint.current_y > 300 && calibPoint.pointNumber == 2)))
	{
		if (calibPoint.handTrackerChecker < numberOfCalibrationPoints)
		{
			calibPoint.handTrackerPointsX[calibPoint.pointNumber][calibPoint.handTrackerChecker] = point.X;
			calibPoint.handTrackerPointsY[calibPoint.pointNumber][calibPoint.handTrackerChecker] = point.Y;;
			calibPoint.handTrackerPointsZ[calibPoint.pointNumber][calibPoint.handTrackerChecker] = distanceToScreen + double(rand() % 1000) / 1000000;
			calibPoint.handTrackerChecker++;
			return;
			
		}
		else
		{
			double masX[11];
			double masY[11];
			double masZ[11];
			for (int i = 0; i < numberOfCalibrationPoints; i++)
			{
				masX[i] = calibPoint.handTrackerPointsX[calibPoint.pointNumber][i];
				masY[i] = calibPoint.handTrackerPointsY[calibPoint.pointNumber][i];
				masZ[i] = calibPoint.handTrackerPointsZ[calibPoint.pointNumber][i];
			}
			sort(masX, masX + numberOfCalibrationPoints);
			sort(masY, masY + numberOfCalibrationPoints);
			sort(masZ, masZ + numberOfCalibrationPoints);
			for (int i = 0; i < numberOfCalibrationPoints; i++)
			{
				printf("Point! %f %f %f\n", masX[i], masY[i], masZ[i]);

			}

			calibPoint.ClickTriger = true;
			calibPoint.A[calibPoint.pointNumber][0] = masX[5];
			calibPoint.A[calibPoint.pointNumber][1] = masY[5];
			calibPoint.A[calibPoint.pointNumber][2] = masZ[5];
			printf("Point! %d \n", calibPoint.pointNumber);
			PlaySoundA("sound.wav", NULL, SND_ASYNC);
			calibPoint.handTrackerChecker = 0;
			calibPoint.current_x = colorpoint.X;
			calibPoint.current_y = colorpoint.Y;
			return;
		}
		//
	}

	else if (correctedDistance > distanceToScreen && calibPoint.ClickTriger == true)
	{
		calibPoint.ClickTriger = false;
		//printf("Click! \n");
		cout << "Click! \n" << endl;
		calibPoint.calibrationPoint[calibPoint.pointNumber] = true;
		calibPoint.pointNumber++;
		return;
	}
}


int main()
{
	//Get window size
	int x_size = GetSystemMetrics(SM_CXSCREEN);
	int y_size = GetSystemMetrics(SM_CYSCREEN);
	
	double distanceToScreen = 1.80;
	double avgDist = 0;
	int numberOfCalibrationPoints = 11;

	ofstream fout;
	fout.open("log.txt");
	if (!fout.is_open()) //if the file was not opened
	{
		cout << "Can't open a file\n"; 
		return 1;
	}
	
	DistanceCorrector dc(
		distanceToScreen,	// meters to the projector
		1010,				// nearest point X
		620					// nearest point Y
	);
	calibPoint cp;
	HRESULT hr = S_OK;
	if (FAILED(GetDefaultKinectSensor(&sensor))) return 0;
	if (sensor) {
		sensor->Open();
		IBodyFrameSource* bodysource = nullptr;
		hr = sensor->get_BodyFrameSource(&bodysource);
		if (FAILED(hr)) return 0;
		hr = bodysource->OpenReader(&bodyReader);
		if (FAILED(hr)) return 0;
		while (1) {
			IBodyFrame* frame = nullptr;
			if (SUCCEEDED(bodyReader->AcquireLatestFrame(&frame))) {

				IBody* Body[BODY_COUNT] = { 0 };
				hr = frame->GetAndRefreshBodyData(BODY_COUNT, Body);
				if (SUCCEEDED(hr))
				{

					for (int count = 0; count < BODY_COUNT; count++)
					{
						BOOLEAN Track = false;
						hr = Body[count]->get_IsTracked(&Track);
						if (SUCCEEDED(hr) && Track)
						{
							//printf("Good \n");
							Joint joint[JointType::JointType_Count];
							hr = Body[count]->GetJoints(JointType::JointType_Count, joint);
							if (SUCCEEDED(hr))
							{
								if (joint[JointType_HandTipRight].TrackingState != TrackingState::TrackingState_NotTracked)
								{
									ICoordinateMapper* coordinateMapper;
									
									hr = sensor->get_CoordinateMapper(&coordinateMapper);
									CameraSpacePoint point = joint[JointType_HandTipRight].Position;
									ColorSpacePoint colorpoint;
									coordinateMapper->MapCameraPointToColorSpace(point, &colorpoint);
									//printf("Point x: %f y: %f z: %f\n", colorpoint.X, colorpoint.Y, point.Z);
									//printf("Corrected Distance %f \n", point.Z);
									double correctedDistance;
									correctedDistance = dc.from(colorpoint.X, colorpoint.Y, point.Z);
									//printf("Corrected Distance %4.3f %4.3f %4.2f \n", point.X, point.Y, correctedDistance);

									
									if (calibration == false)
									{
										if (cp.calibrationPoint[0] == false && cp.calibrationPoint[1] == false && cp.calibrationPoint[2] == false && cp.calibrationPoint[3] == false) {


											point_calibration(correctedDistance, distanceToScreen, numberOfCalibrationPoints, colorpoint, point, cp);
											 
										}
										if (cp.calibrationPoint[0] == true && cp.calibrationPoint[1] == false && cp.calibrationPoint[2] == false && cp.calibrationPoint[3] == false) {
											
											point_calibration(correctedDistance, distanceToScreen, numberOfCalibrationPoints, colorpoint, point, cp);

										}
										if (cp.calibrationPoint[0] == true && cp.calibrationPoint[1] == true && cp.calibrationPoint[2] == false && cp.calibrationPoint[3] == false) {
											
											point_calibration(correctedDistance, distanceToScreen, numberOfCalibrationPoints, colorpoint, point, cp);

										}
										if (cp.calibrationPoint[0] == true && cp.calibrationPoint[1] == true && cp.calibrationPoint[2] == true && cp.calibrationPoint[3] == false) {

											
											point_calibration(correctedDistance, distanceToScreen, numberOfCalibrationPoints, colorpoint, point, cp);

										}
										if (cp.calibrationPoint[0] == true && cp.calibrationPoint[1] == true && cp.calibrationPoint[2] == true && cp.calibrationPoint[3] == true) {
											
											
											for (int j = 0; j < 4; j++)
											{
												fout << "Point " << j << endl;
												for (size_t i = 0; i < numberOfCalibrationPoints; i++)
												{
													fout << "X " << cp.handTrackerPointsX[j][i] << "Y " << cp.handTrackerPointsY[j][i] << "Z " << cp.handTrackerPointsZ[j][i] << endl;
												}
											}

											//compute the calibration 
											double B[4][2] = {
											{ 0, 0},
											{ x_size, 0},
											{0, y_size},
											{ x_size, y_size },
											};		// calibration screen matrix

											
											for (int i = 0; i < 4; i++) 
											{
												for (int j = 0; j < 3; j++)
												{
													A[i][j] = cp.A[i][j];
												}
											}
											
											for (int i = 0; i < 4; i++){
												printf("X:%f Y:%f Z:%f \n", A[i][0], A[i][1],A[i][2]);
												k2p.addCalibPoint(A[i][0], A[i][1], A[i][2], B[i][0], B[i][1]);
												avgDist += A[i][2];
												fout << "X: " << A[i][0] << "Y: " << A[i][1] << "Z: " << A[i][2] << endl;
											}
											avgDist /= 4;
											k2p.calibrate();

											calibration = true;
											printf("Calibration done! \n");
											fout << "    X     " << "   Y   " << "   Z0   " << "   Zcor   " << endl;
										}
									}
									if (calibration == true)
									{
										if (correctedDistance < distanceToScreen)
										{
											k2p.map(point.X, point.Y, avgDist, screen_x, screen_y);
											//printf("X:%4.2f Y:%4.2f Z:%4.2f --- X:%4.0f Y:%4.0f\n", point.X, point.Y, correctedDistance, screen_x, screen_y);
											//Point_trigger = true;
											if (screen_y > 50 && screen_y < (y_size-50))
											{
												SetCursorPos(screen_x, screen_y);
												fout << point.X << " " << point.Y << " " << point.Z << " " << avgDist << endl;
												Point_trigger = true;
												dx = screen_x;
												dy = screen_y;
											}
										}


										else if (correctedDistance > (distanceToScreen + 0.05) && Point_trigger == true)
										{
												Point_trigger = false;
												PlaySoundA("sound.wav", NULL, SND_ASYNC);
												printf("Click! \n");
												
												mouse_event(MOUSEEVENTF_LEFTDOWN, dx, dy, 0, 0);
												mouse_event(MOUSEEVENTF_LEFTUP, dx, dy, 0, 0);
										}

									}
								}
							}
						}
						SafeRelease(Body[count]);
					}
					
				}
				
			}
			SafeRelease(frame);
		}
	}
	fout.close();
	return 0;
}
