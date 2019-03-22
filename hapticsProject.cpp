// (C) 2001-2015 Force Dimension
// All Rights Reserved.
//
// Version 3.6.0

// File edited by Rajiv Mantena and Keith Dockstader 
// to impliment Wave Variable approch in Falcon-Falcon teleoperation
// at the University of Utah
// in the Spring of 2017
// for the Haptics ME 7960 course
// mantenarajiv@gmail.com
// 


#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <queue>
#include <iostream>

#include "drdc.h"

#define DEFAULT_K 200.0
#define MIN_SCALE 0.2
#define MAX_SCALE 5.0

#define MIN(a,b) ((a)<(b))?(a):(b)
#define MAX(a,b) ((a)>(b))?(a):(b)

//
// Custom Functions to Add and Subtract two 3x1 Vectors 
// 
std::vector <double> sum(std::vector <double> a, std::vector <double> b) {
	std::vector <double> c = { 0,0,0 };
	c.at(0) = a.at(0) + b.at(0);
	c.at(1) = a.at(1) + b.at(1);
	c.at(2) = a.at(2) + b.at(2);

	return c;
}

std::vector <double> sub(std::vector <double> a, std::vector <double> b) {
	std::vector <double> c = { 0,0,0 };
	c.at(0) = a.at(0) - b.at(0);
	c.at(1) = a.at(1) - b.at(1);
	c.at(2) = a.at(2) - b.at(2);

	return c;
}

// Custom Function to Multiply a 3x1 Vector with a Scalar (double)

std::vector <double> mult(double a, std::vector<double> b) {
	std::vector <double> c = { 0,0,0 };
	c.at(0) = a*b.at(0);
	c.at(1) = a*b.at(1);
	c.at(2) = a*b.at(2);

	return c;
}

void printHeader() {
	int major, minor, release, revision;
	dhdGetSDKVersion(&major, &minor, &release, &revision);

	printf("\n                          Haptics ME 7690 Project | Spring 2017 | University of Utah    \n");
	printf("                      Original Authors : Force Dimension - Master Slave Example %d.%d.%d.%d\n", major, minor, release, revision);
	printf("                                         (C) 2001-2015 Force Dimension\n");
	printf("\n                                Code Edited by Rajiv Mantena and Keith Dockstader\n");
	printf("                  ----------------------------------------------------------------------------\n\n");
}

void printFirst() {
	system("cls");
	printHeader();
	printf("                       Bilateral Position Servo WITHOUT Time Delay is being run currently \n");
	printf("\n                       Press 'y' to switch to Bilateral Position Servo WITH Time Delay \n");
	printf("                     Press 'u' to switch to Bilateral Teleoperation using Wave Variables \n");
	printf("\n     Press 'q' to QUIT    |    Press 'a'/'s' to increase/decrease Kp    |    Press 'd'/'f' to increase/decrease Kd    \n");
	printf("\n                                        Press 'e' for EMERGENCY STOP !!!    \n");
	printf("\n                  ----------------------------------------------------------------------------\n");
}

void printSecond() {
	system("cls");
	printHeader();
	printf("                       Bilateral Position Servo WITH Time Delay is being run currently \n");
	printf("\n                     Press 't' to switch to Bilateral Position Servo WITHOUT Time Delay \n");
	printf("                     Press 'u' to switch to Bilateral Teleoperation using Wave Variables \n");
	printf("\n				                  Select time delay :         \n");
	printf("       Press 'c' for 0.001 secs  |  Press 'v' for 0.01 secs  |  Press 'b' for 0.1 secs  |  Press 'n' for 1 sec     \n");
	printf("\n     Press 'q' to QUIT    |    Press 'a'/'s' to increase/decrease Kp    |    Press 'd'/'f' to increase/decrease Kd    \n");
	printf("\n                  ----------------------------------------------------------------------------\n");
}

void printThird() {
	system("cls");
	printHeader();
	printf("                     Bilateral using Wave Variables with time delay is being run currently \n");
	printf("\n                     Press 't' to switch to Bilateral Position Servo WITHOUT Time Delay \n");
	printf("                         Press 'y' to switch to Bilateral Position Servo WITH Time Delay \n");
	printf("\n				                  Select time delay :         \n");
	printf("       Press 'c' for 0.001 secs  |  Press 'v' for 0.01 secs  |  Press 'b' for 0.1 secs  |  Press 'n' for 1 sec     \n");
	printf("\n     Press 'q' to QUIT    |    Press 'a'/'s' to increase/decrease Kp    |    Press 'd'/'f' to increase/decrease Kd    \n");
	printf("\n                  ----------------------------------------------------------------------------\n");
}

void reCalcGains(double &K, double &Ke, double &A, double &B, double &C) {
	double tau = 0.0016;
	double T = 0.001;
	A = (2 * Ke + K*T) / (2 * tau + 1 * T);
	B = -1*(-2 * Ke + K*T) / (2 * tau + 1 * T);
	// C = 1;
	C = (-2 * tau + 1 * T) / (2 * tau + 1 * T);
}

// Main Function
int myProgram(int controlStrategy = 1, double timeDelay = 0.01)
{
	double mx, my, mz;
	double sx, sy, sz;
	double sx_d = 0, sy_d = 0, sz_d = 0;
	double mx_d = 0, my_d = 0, mz_d = 0;

	double fx = 0, fy = 0, fz = 0;
	double fx_slave_prev = 0, fy_slave_prev = 0, fz_slave_prev = 0;
	double fx_master_prev = 0, fy_master_prev = 0, fz_master_prev = 0;
	double currError_x = 0, currError_y = 0, currError_z = 0;
	double prevError_x = 0, prevError_y = 0, prevError_z = 0;
	double prevError_x_m = 0, prevError_y_m = 0, prevError_z_m = 0;
	double prevError_x_s = 0, prevError_y_s = 0, prevError_z_s = 0;

	double refTime = dhdGetTime();
	int done = 0;
	int master, slave;
	int tempCount = 0;
	// The below variable is used to scale up the X forces 
	// There is a lot of natural damping in motion about x-axis in the robot, this has to be compensated for
	double x_scale = 1.5;

	std::queue<std::vector<double>> posHistoryMaster;
	std::queue<std::vector<double>> posHistorySlave;
	std::queue<std::vector<double>> us_queue;
	std::queue<std::vector<double>> um_queue;
	std::vector<double> newPos;
	bool passedDelay = false;

	std::vector<double> xm = { 0,0,0 };
	std::vector<double> xs = { 0,0,0 };
	std::vector<double> xm_dot = { 0,0,0 };
	std::vector<double> xs_dot = { 0,0,0 };

	std::vector<double> um_calc = { 0,0,0 };
	std::vector<double> us_calc = { 0,0,0 };
	std::vector<double> xtilde_m = { 0,0,0 };
	std::vector<double> xtilde_s = { 0,0,0 };
	
	std::vector<double> xtilde_m_prev = { 0,0,0 };
	std::vector<double> xtilde_s_prev = { 0,0,0 };

	std::vector<double> xsd_dot = { 0,0,0 };
	std::vector<double> xmd_dot = { 0,0,0 };
	std::vector<double> vm = { 0,0,0 };
	std::vector<double> vs = { 0,0,0 };
	std::vector<double> xmd = { 0,0,0 };
	std::vector<double> xsd = { 0,0,0 };
	std::vector<double> Fs = { 0,0,0 };
	std::vector<double> Fm = { 0,0,0 };

	/*
	// Select Controller Strategy & time delays
	//
	// Control Strategies	: 1 - Bilateral Teleoperation (Position Servo) with no time delays (clean code)
	//                        2 - Bilateral Teleoperation (Position Servo) with time delays
	//						  3 - Wave Variable based Bilateral Teleoperation
							  4 - Bilateral Teleoperation (Position Servo) with no time delays V2 (Super clean code)
	//
	// Time Delay			: Specify the desired time delay in secs.
	// K					: P Gain
	// Ke					: D Gain
	// b					: Wave Variable Compensator
	*/
	// int controlStrategy = 1;
	// double timeDelay = 0.001;

	// Default Z transform Controller Gains;
	double myA = 0;
	double myB = 0;
	double myC = 0;
	double C = 0.0;

	// Default S transform Controller Gains;
	double K = 1500, Ke = 5;
	double b = 10;
	double ks = K, km = K, bs = b, bm = b;

	double time, myTime, initTime, prevtime, dt = 0.06, timeNow = 0, currTime;

	{
		// open and initialize 2 devices
		for (int dev = 0; dev < 2; dev++) {

			// open device
			if (drdOpenID(dev) < 0) {
				printf("error: not enough devices found\n");
				dhdSleep(2.0);
				for (int j = 0; j <= dev; j++) drdClose(j);
				return -1;
			}

			// exclude some device types that have not been fully tested with 'mirror'
			bool incompatible = false;
			switch (dhdGetSystemType()) {
			case DHD_DEVICE_SIGMA331:
			case DHD_DEVICE_SIGMA331_LEFT:
				incompatible = true;
				break;
			}

			// check that device is supported
			if (incompatible || !drdIsSupported()) {
				printf("error: unsupported device (%s)\n", dhdGetSystemName(dev));
				dhdSleep(2.0);
				for (int j = 0; j <= dev; j++) drdClose(j);
				return -1;
			}

			// initialize Falcon by hand if necessary
			if (!drdIsInitialized() && dhdGetSystemType() == DHD_DEVICE_FALCON) {
				printf("please initialize Falcon device...\r"); fflush(stdout);
				while (!drdIsInitialized()) dhdSetForce(0.0, 0.0, 0.0);
				printf(" \r");
				dhdSleep(0.5);
			}

			// initialize if necessary
			if (!drdIsInitialized(dev) && (drdAutoInit(dev) < 0)) {
				printf("error: initialization failed (%s)\n", dhdErrorGetLastStr());
				dhdSleep(2.0);
				for (int j = 0; j <= dev; j++) drdClose(j);
				return -1;
			}

			// start robot control loop
			if (drdStart(dev) < 0) {
				printf("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr());
				dhdSleep(2.0);
				for (int j = 0; j <= dev; j++) drdClose(j);
				return -1;
			}
		}

		// default role assignment
		master = 1;
		slave = 0;
		{
			// prefer Falcon as master
			if (dhdGetSystemType(0) != DHD_DEVICE_FALCON && dhdGetSystemType(1) == DHD_DEVICE_FALCON) {
				master = 1;
				slave = 0;
			}

			// give preference to omega.3 as slave
			if (dhdGetSystemType(0) == DHD_DEVICE_OMEGA3 && dhdGetSystemType(1) != DHD_DEVICE_OMEGA3) {
				master = 1;
				slave = 0;
			}

			// if a device is virtual, make it the master
			if (dhdGetComMode(1) == DHD_COM_MODE_VIRTUAL) {
				master = 1;
				slave = 0;
			}
		}


		/* Below 5 lines of code are just to print the Serial numbers of Master and Slave */

		// ushort mastersn, slavesn;
		// dhdGetSerialNumber(&mastersn, master);
		// dhdGetSerialNumber(&slavesn, slave);
		// printf("%s haptic device [sn: %04d] as master\n", dhdGetSystemName(master), mastersn);
		// printf("%s haptic device [sn: %04d] as slave\n", dhdGetSystemName(slave), slavesn);

		// display instructions
		// printf("\n");
		// printf("press 'd' to decrease scaling factor\n");
		// printf(" 'u' to increase scaling factor\n");
		// printf(" ',' to decrease virtual stiffness\n");
		// printf(" '.' to increase virtual stiffness\n");
		// printf(" 'q' to quit\n\n");

		// center both devices
		// drdMoveToPos(0.0, 0.0, 0.0, false, master);
		drdMoveToPos(0.0, 0.0, 0.0, true, master);
		drdMoveToPos(0.0, 0.0, 0.0, true, slave);
		while (drdIsMoving(master) || drdIsMoving(slave)) drdWaitForTick(master);

		// stop regulation on master, stop motion filters on slave
		drdStop(true, master);
		dhdSetForce(0.0, 0.0, 0.0, master);

		drdStop(true, slave);
		dhdSetForce(0.0, 0.0, 0.0, slave);
		drdEnableFilter(false, slave);
	}

	// Below 2 lines of code are to calculate dt
	initTime = dhdGetTime();
	prevtime = refTime;
	reCalcGains(K, Ke, myA, myB, myC);

	/*-------------------------------------------------------------------------------------------------------*
	*		Bilateral Teleoperation (Position Servo) without time delay | Simplest code possible			 *
	*--------------------------------------------------------------------------------------------------------*/
	if (controlStrategy == 1) {
		printFirst();
		while (!done) {
			tempCount++;

			dhdGetPosition(&mx, &my, &mz, master);

			dhdGetPosition(&sx, &sy, &sz, slave);

			currError_x = mx - sx;	currError_y = my - sy;	currError_z = mz - sz;

			fx = myA * currError_x - myB*prevError_x - myC*fx_master_prev;
			fy = myA * currError_y - myB*prevError_y - myC*fy_master_prev;
			fz = myA * currError_z - myB*prevError_z - myC*fz_master_prev;

			fx_master_prev = fx;  fy_master_prev = fy;  fz_master_prev = fz;

			prevError_x = currError_x;	prevError_y = currError_y;	prevError_z = currError_z;

			dhdSetForce(fx*x_scale, fy, fz, slave);
			dhdSetForce(-fx*x_scale, -fy, -fz, master);

			myTime = dhdGetTime();
			dt = myTime - prevtime;
			prevtime = myTime;

			// print stats and check for exit condition
			time = dhdGetTime();

			if (time - refTime > 0.04) {
				if (tempCount < 1000) {
					printf("                                     Master & Slave are READY to Teleoperate                          \r");
					// printf("       Gains : %f,     %f,      %f,      %f,     %f    \n", K, Ke, myA, myB, myC);
				}
				else printf("                                   K = %04d | Ke = %02d | Freqency : %0.1f Hz             \r", (int)K, int(Ke), 1 / dt);
				refTime = time;
				// if (!drdIsRunning(slave)) done = -1;
				if (dhdKbHit()) {
					switch (dhdKbGet()) {
					case 'q': return 0;
					case 'e': return 99;
					case 's': K -= 0.1*K; reCalcGains(K,Ke,myA,myB,myC); break;
					case 'a': K += 0.1*K; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'f': Ke -= 0.1*Ke; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'd': Ke += 0.1*Ke; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'y': return 2;
					case 'u': return 3;

					}
				}
			}
		}
	}

	/*-------------------------------------------------------------------------------------------------------*
	*		                 Bilateral Teleoperation (Position Servo) with time delay 			             *
	*--------------------------------------------------------------------------------------------------------*/
	if (controlStrategy == 2) {

		printSecond();
		while (!done) {
			tempCount++;

			dhdGetPosition(&mx, &my, &mz, master);
			dhdGetPosition(&sx, &sy, &sz, slave);

			posHistoryMaster.push({ mx,my,mz });
			posHistorySlave.push({ sx,sy,sz });

			if (passedDelay) {
				newPos = posHistoryMaster.front();
				posHistoryMaster.pop();
				mx_d = newPos.at(0); my_d = newPos.at(1); mz_d = newPos.at(2);
			}

			currError_x = mx_d - sx;	currError_y = my_d - sy;	currError_z = mz_d - sz;

			fx = myA * currError_x - myB*prevError_x_s - myC*fx_slave_prev;
			fy = myA * currError_y - myB*prevError_y_s - myC*fy_slave_prev;
			fz = myA * currError_z - myB*prevError_z_s - myC*fz_slave_prev;

			prevError_x_s = currError_x;	prevError_y_s = currError_y;	prevError_z_s = currError_z;
			fx_slave_prev = fx;  fy_slave_prev = fy;  fz_slave_prev = fz;
			dhdSetForce(fx*x_scale, fy, fz, slave);

			if (passedDelay) {
				newPos = posHistorySlave.front();
				posHistorySlave.pop();
				sx_d = newPos.at(0); sy_d = newPos.at(1); sz_d = newPos.at(2);
			}

			currError_x = mx - sx_d;	currError_y = my - sy_d;	currError_z = mz - sz_d;

			fx = myA * currError_x - myB*prevError_x_m - myC*fx_master_prev;
			fy = myA * currError_y - myB*prevError_y_m - myC*fy_master_prev;
			fz = myA * currError_z - myB*prevError_z_m - myC*fz_master_prev;

			prevError_x_m = currError_x;	prevError_y_m = currError_y;	prevError_z_m = currError_z;
			fx_master_prev = fx;  fy_master_prev = fy;  fz_master_prev = fz;
			dhdSetForce(-fx*x_scale, -fy, -fz, master);

			myTime = dhdGetTime();
			dt = myTime - prevtime;
			prevtime = myTime;

			// print stats and check for exit condition
			time = dhdGetTime();

			if (!passedDelay) {
				if (posHistoryMaster.size() > timeDelay * 1000)
				{
					passedDelay = true;
				}
			}

			if (time - refTime > 0.04) {

				if (tempCount < 1000) {
					printf("                                     Master & Slave are READY to Teleoperate                          \r");
				}
				else printf("              K = %04d | Ke = %02d | Delay %0.03f Secs (%d) | Delay Passed ? : %s | Freqency %0.1f Hz             \r",
					(int)K, int(Ke), timeDelay, posHistoryMaster.size(), passedDelay ? "yes" : "no", 1 / dt);
				refTime = time;
				// if (!drdIsRunning(slave)) done = -1;
				if (dhdKbHit()) {
					switch (dhdKbGet()) {
					case 'q': return 0;
					case 'e': return 99;
					case 's': K -= 0.1*K; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'a': K += 0.1*K; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'f': Ke -= 0.1*Ke; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'd': Ke += 0.1*Ke; reCalcGains(K, Ke, myA, myB, myC); break;
					case 't': return 1;
					case 'u': return 3;
					case 'c': return 21;
					case 'v': return 22;
					case 'b': return 23;
					case 'n': return 24;
					}
				}
			}
		}
	}

	/*-------------------------------------------------------------------------------------------------------*
	*		               Bilateral Teleoperation using Wave Variables with time delay  			         *
	*--------------------------------------------------------------------------------------------------------*/
	if (controlStrategy == 3) {

		printThird();
		while (!done) {
			tempCount++;
			ks = K; km = K; bs = b; bm = b;

			dhdGetPosition(&xm.at(0), &xm.at(1), &xm.at(2), master);
			dhdGetPosition(&xs.at(0), &xs.at(1), &xs.at(2), slave);

			dhdGetLinearVelocity(&xm_dot.at(0), &xm_dot.at(1), &xm_dot.at(2), master);
			dhdGetLinearVelocity(&xs_dot.at(0), &xs_dot.at(1), &xs_dot.at(2), slave);

			// Get um and us and add them to a queue
			um_calc.at(0) = (km*xtilde_m.at(0) + bm*xm_dot.at(0) + (b - bm)*xmd_dot.at(0)) / sqrt(2 * b);
			um_calc.at(1) = (km*xtilde_m.at(1) + bm*xm_dot.at(1) + (b - bm)*xmd_dot.at(1)) / sqrt(2 * b);
			um_calc.at(2) = (km*xtilde_m.at(2) + bm*xm_dot.at(2) + (b - bm)*xmd_dot.at(2)) / sqrt(2 * b);
			um_queue.push(um_calc);

			us_calc.at(0) = (-1 * (ks*xtilde_s.at(0) + bs*xs_dot.at(0) + (b - bs)*xsd_dot.at(0))) / sqrt(2 * b);
			us_calc.at(1) = (-1 * (ks*xtilde_s.at(1) + bs*xs_dot.at(1) + (b - bs)*xsd_dot.at(1))) / sqrt(2 * b);
			us_calc.at(2) = (-1 * (ks*xtilde_s.at(2) + bs*xs_dot.at(2) + (b - bs)*xsd_dot.at(2))) / sqrt(2 * b);
			us_queue.push(us_calc);

			// Get vm and vs if they are ready
			if (passedDelay) {
				vm = us_queue.front();
				us_queue.pop();
			}

			if (passedDelay) {
				vs = um_queue.front();
				um_queue.pop();
			}

			xmd.at(0) += xmd_dot.at(0)*dt;
			xmd.at(1) += xmd_dot.at(1)*dt;
			xmd.at(2) += xmd_dot.at(2)*dt;

			xsd.at(0) += xsd_dot.at(0)*dt;
			xsd.at(1) += xsd_dot.at(1)*dt;
			xsd.at(2) += xsd_dot.at(2)*dt;

			// Calclate desured velocities from wave variables
			xmd_dot.at(0) = (km*xtilde_m.at(0) + bm*xm_dot.at(0) - sqrt(2 * b)*vm.at(0)) / (b + bm);
			xmd_dot.at(1) = (km*xtilde_m.at(1) + bm*xm_dot.at(1) - sqrt(2 * b)*vm.at(1)) / (b + bm);
			xmd_dot.at(2) = (km*xtilde_m.at(2) + bm*xm_dot.at(2) - sqrt(2 * b)*vm.at(2)) / (b + bm);

			xsd_dot.at(0) = (ks*xtilde_s.at(0) + bs*xs_dot.at(0) + sqrt(2 * b)*vs.at(0)) / (b + bs);
			xsd_dot.at(1) = (ks*xtilde_s.at(1) + bs*xs_dot.at(1) + sqrt(2 * b)*vs.at(1)) / (b + bs);
			xsd_dot.at(2) = (ks*xtilde_s.at(2) + bs*xs_dot.at(2) + sqrt(2 * b)*vs.at(2)) / (b + bs);

			xtilde_s = sub(xs, xsd);
			xtilde_m = sub(xm, xmd);

			Fm.at(0) = myA*xtilde_m.at(0) - myB*xtilde_m_prev.at(0) - myC*fx_master_prev;
			Fm.at(1) = myA*xtilde_m.at(1) - myB*xtilde_m_prev.at(1) - myC*fy_master_prev;
			Fm.at(2) = myA*xtilde_m.at(2) - myB*xtilde_m_prev.at(2) - myC*fz_master_prev;

			Fs.at(0) = myA*xtilde_s.at(0) - myB*xtilde_s_prev.at(0) - myC*fx_slave_prev;
			Fs.at(1) = myA*xtilde_s.at(1) - myB*xtilde_s_prev.at(1) - myC*fy_slave_prev;
			Fs.at(2) = myA*xtilde_s.at(2) - myB*xtilde_s_prev.at(2) - myC*fz_slave_prev;

			xtilde_m_prev = xtilde_m;
			xtilde_s_prev = xtilde_s;

			fx_master_prev = Fm.at(0);  fy_master_prev = Fm.at(1);  fz_master_prev = Fm.at(2);

			fx_slave_prev = Fs.at(0);  fy_slave_prev = Fs.at(1);  fz_slave_prev = Fs.at(2);

			// Set the force to act on the robot
			dhdSetForce(-Fm.at(0)*x_scale, -Fm.at(1), -Fm.at(2), master);
			dhdSetForce(-Fs.at(0)*x_scale, -Fs.at(1), -Fs.at(2), slave);
			
			// dt calculation
			myTime = dhdGetTime();
			dt = myTime - prevtime;
			prevtime = myTime;

			if (!passedDelay) {
				if (um_queue.size() > timeDelay * 1000)
				{
					passedDelay = true;
				}
			}

			// print stats and check for exit condition
			time = dhdGetTime();
			if (time - refTime > 0.04) {
				if (tempCount < 1000) {
					printf("                                     Master & Slave are READY to Teleoperate                          \r");
				}
				else printf("              K = %04d | Ke = %02d | Delay %0.02f Secs (%d) | Delay Passed ? : %s | Freqency %0.1f Hz             \r",
					(int)K, int(Ke), timeDelay, us_queue.size(), passedDelay ? "yes" : "no", 1 / dt);
				refTime = time;
				// if (!drdIsRunning(slave)) done = -1;
				if (dhdKbHit()) {
					switch (dhdKbGet()) {
					case 'q': return 0;
					case 'e': return 99;
					case 's': K -= 0.1*K; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'a': K += 0.1*K; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'f': Ke -= 0.1*Ke; reCalcGains(K, Ke, myA, myB, myC); break;
					case 'd': Ke += 0.1*Ke; reCalcGains(K, Ke, myA, myB, myC); break;
					case 't': return 1;
					case 'y': return 2;
					case 'c': return 31;
					case 'v': return 32;
					case 'b': return 33;
					case 'n': return 34;
					}
				}
			}
		}
	}

	// report exit cause
	printf(" \r");
	if (done == -1) printf("\nregulation finished abnormally on slave device\n");
	else printf("\nexiting on user request\n");

	// close the connection
	drdClose(slave);
	drdClose(master);

	// exit
	printf("\ndone.\n");
	return 0;
}

// Main Function
int main(int argc,
	char **argv) {
	
	int returnVal = -2;

	while (1) {
		system("cls");
		printHeader();

		if (returnVal == 0) break;
		else if (returnVal == -2) returnVal = myProgram(1);
		else if (returnVal == 2) returnVal = myProgram(2);
		else if (returnVal == 3) returnVal = myProgram(3);
		else if (returnVal == 21) returnVal = myProgram(2, 0.001);
		else if (returnVal == 22) returnVal = myProgram(2, 0.01);
		else if (returnVal == 23) returnVal = myProgram(2, 0.1);
		else if (returnVal == 24) returnVal = myProgram(2, 1);
		else if (returnVal == 31) returnVal = myProgram(3, 0.001);
		else if (returnVal == 32) returnVal = myProgram(3, 0.01);
		else if (returnVal == 33) returnVal = myProgram(3, 0.1);
		else if (returnVal == 34) returnVal = myProgram(3, 1);
		else if (returnVal == 99) {
			system("cls");
			printf("\n\n\n\n\n\n\n                                                      Emergency Stop !!               \n");
			printf("\n                                                     ... please wait ...\n");
			drdSleep(5);
			returnVal = -2;
		}
		else {
			system("cls");
			printf("\n\n\n                                             Invalid Input                                               \n");
			drdSleep(5);
			returnVal = -2;
		}
	}

	printf("                                                       Thank You                             ");
	drdSleep(2);
	return 0;
}