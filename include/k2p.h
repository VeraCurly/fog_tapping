#pragma once

/* @summary Maps Kinect (x, y) points with a distance (z) into 2D projection points.
 * Requires 4 calls of the method "addCalibPoint" and a subsequent call of "calibrate" method
 * before the method "map" can be called.
 */
class Kinect2Projector
{
public:
	/* @description Constructor */
	Kinect2Projector();

	/* @description Destructor */
	virtual ~Kinect2Projector();

	/* @description Adds a calibration point
	 * @param(kx) Kinect point X coordinate
	 * @param(ky) Kinect point Y coordinate
	 * @param(kz) Kinect point Z coordinate
	 * @param(px) Projector point X coordinate
	 * @param(py) Projector point Y coordinate
	 * @returns True if point added successfully, or false if 4 the calibration was done 
	 */
	bool addCalibPoint(double kx, double ky, double kz, double px, double py);

	/* @description Computes the mapping matrix (solves linear equations)
	 * @returns True if matrix was computed, or false if there are less than 4 calibration points provided
	 */
	bool calibrate();

	/* @description Maps a Kinect point with a distance into a 2D projection point
	 * @param(kx) Kinect point X coordinate
	 * @param(ky) Kinect point Y coordinate
	 * @param(kz) Kinect point Z coordinate
	 * @param(px) Projector point X coordinate
	 * @param(py) Projector point Y coordinate
	 * @returns True if the mapping was successful, or false if there are no mapping matrix computed yet.
	 */
	bool map(double kx, double ky, double kz, double& px, double& py);

	/* @description Set it to "true" to have some computation output into stdout from the class methods */
	static bool VERBOSE;

private:
	double* calibInput;
	double* calibOutput;
	double* t;

	int pointID;
};