#pragma once

/* @summary Converts Kinect distance from point-to-plane to plane-to-plane */
class DistanceCorrector
{
public:
	/* @description Constructor
	 * @param(distanceToScreen) Distance to screen in meters
	 * @param(centerX) Kinect-to-screen nearest point X pcoordinate in pixels
	 * @param(centerY) Kinect-to-screen nearest point Y pcoordinate in pixels
	 * @param(kinectCameraAngleX) Kinect camera width angle in degrees
	 * @param(kinectCameraAngleY) Kinect camera height angle in degrees
	 * @param(kinectResolutionX) Kinect 2D resolution width in pixels
	 * @param(kinectResolutionY) Kinect 2D resolution height in pixels
	 */
	DistanceCorrector(
		double distanceToScreen,
		double centerX,
		double centerY,
		double kinectCameraAngleX = 57,
		double kinectCameraAngleY = 43,
		double kinectResolutionX = 1366,
		double kinectResolutionY = 720
	);

	/* @description Converts the distance given the kinect 3D point
	 * @param(x) Kinect point X coordinate
	 * @param(y) Kinect point Y coordinate
	 * @param(z) Kinect point Z coordinate (distance)
	 * @returns Plane-to-plane distance
	 */
	double from(double x, double y, double z);

private:
	double distanceToScreen;
	double centerX;
	double centerY;

	double kinectCameraAngleX;
	double kinectCameraAngleY;
	double kinectResolutionX;
	double kinectResolutionY;

	double resolutionInMetersX;
	double resolutionInMetersY;
};

