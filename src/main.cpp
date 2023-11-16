#include "main.h"
#include "lemlib/api.hpp"
#include "Graphy/Grapher.hpp"
#include "definitions.hpp"

std::shared_ptr<graphy::AsyncGrapher> grapher(new graphy::AsyncGrapher("workpls"), 25 * okapi::millisecond);


float lat_kP = 1;
float lat_kD = 1;
float ang_kP = 1;
float ang_kD = 1;

bool lastKnowStateUp;
bool lastKnowStateDown;
bool lastKnowStateLeft;
bool lastKnowStateRight;

bool lastKnowStateX;
bool lastKnowStateY;
bool lastKnowStateA;
bool lastKnowStateB;

bool lastKnowStateR1;
bool lastKnowStateL1;


/*Device Initalizations*/

	/*Drivetrain Initializations*/
	pros::Motor lD1(LD1, SPEEDBOX, true);
	pros::Motor lD2(LD2, SPEEDBOX, true);
	pros::Motor lD3(LD3, SPEEDBOX, true);
	pros::Motor rD1(RD1, SPEEDBOX, false);
	pros::Motor rD2(RD2, SPEEDBOX, false);
	pros::Motor rD3(RD3, SPEEDBOX, false);

	pros::MotorGroup lDrive({lD1, lD2, lD3});
	pros::MotorGroup rDrive({rD1, rD2, rD3});

	pros::Imu imu(IMU_PORT);

	pros::Rotation odomRot(ODOM_ROT, true);

	lemlib::TrackingWheel odomWheel(&odomRot, 2.75, 0, 1);


	/*Controller Initialization*/
	pros::Controller master(pros::E_CONTROLLER_MASTER);

/*End of Device Initalizations*/


/*LemLib Drivetrain Initilization*/
lemlib::Drivetrain drivetrain
{
	&lDrive, /*Pointer to the left drive motor group*/
	&rDrive, /*Pointer to the right drive motor group*/
	10.5, /*Track Width*/
	3.25, /*Wheel Diameter*/
	450, /*Wheel RPM*/
	8 /*Chase Power*/
};

/*LemLib Odometery Sensors Initilization*/
lemlib::OdomSensors odomSensors
{
	&odomWheel, // left encoder
	nullptr, // right encoder
	nullptr, // back encoder
	nullptr, // front encoder
	&imu // imu
};

/*Lateral (Forwards/Backwards) PID Initilization*/
lemlib::ControllerSettings lateralController 
{
    lat_kP,  //30, // kP
    lat_kD, //50, // kD
    1, // smallErrorRange
    100000, // smallErrorTimeout
    3, // largeErrorRange
    500000, // largeErrorTimeout
	10 // Slew Rate
};

/*Angular (Turning) PID Initilization*/
lemlib::ControllerSettings angularController 
{
    ang_kP, // kP
    ang_kD, // kD
    1, // smallErrorRange
    100000, // smallErrorTimeout
    3, // largeErrorRange
    500000, // largeErrorTimeout
	10 // Slew Rate
};

/*LemLib Chassis Initilization*/
lemlib::Chassis drive(drivetrain, lateralController, angularController, odomSensors);


/*Function Definitions*/

	void screen() 
	{
    	// loop forever
    	while (true) 
		{
        	lemlib::Pose pose = drive.getPose(); // get the current position of the robot
        	pros::lcd::print(0, "x: %f", pose.x); // print the x position
       		pros::lcd::print(1, "y: %f", pose.y); // print the y position
        	pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
			pros::lcd::print(3, "lat_kP: %f", lat_kP);
			pros::lcd::print(4, "lat_kD: %f", lat_kD);
			pros::lcd::print(5, "ang_kP: %f", ang_kP);
			pros::lcd::print(6, "ang_kD: %f", ang_kD);
        	pros::delay(10);
    	}
}
	

	/*Chassis Functions*/
	void drve()
	{
		drive.tank(master.get_analog(LeftY), master.get_analog(RightY), 7);
	}

	
	





/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	drive.calibrate();
	lDrive.set_brake_modes(HOLD);
	rDrive.set_brake_modes(HOLD);
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() 
{

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() 
{

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() 
{
	
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() 
{

	grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);
	
	grapher->startTask();

	while (true)
	{
		
		grapher->update("Actual Vel", drive.getPose().y);

		if (master.get_digital(Up) != lastKnowStateUp)
		{
			lastKnowStateUp = master.get_digital(Up);
			if (master.get_digital(Up))
			{
				lat_kP += 1;
			}
			
		}

		if (master.get_digital(Down) != lastKnowStateDown)
		{
			lastKnowStateDown = master.get_digital(Down);
			if (master.get_digital(Down))
			{
				lat_kP -= 1;
			}
		
		}

		if (master.get_digital(Right) != lastKnowStateRight)
		{
			lastKnowStateRight = master.get_digital(Right);
			if (master.get_digital(Right))
			{
				ang_kP += 1;
			}
			
		}

		if (master.get_digital(Left) != lastKnowStateLeft)
		{
			lastKnowStateLeft = master.get_digital(Left);
			if (master.get_digital(Left))
			{
				ang_kP -= 1;
			}
			
		}

		if (master.get_digital(X) != lastKnowStateX)
		{
			lastKnowStateX = master.get_digital(X);
			if (master.get_digital(X))
			{
				lat_kD += 1;
			}
			
		}


		if (master.get_digital(B) != lastKnowStateB)
		{
			lastKnowStateB = master.get_digital(B);
			if (master.get_digital(B))
			{
				lat_kD -= 1;
			}
			
		}

		if(master.get_digital(A) != lastKnowStateA)
		{
			lastKnowStateA = master.get_digital(A);
			if (master.get_digital(A))
			{
				ang_kD += 1;
			}
			
		}

		if (master.get_digital(Y) != lastKnowStateY)
		{
			lastKnowStateY = master.get_digital(Y);
			if (master.get_digital(Y))
			{
				ang_kP -= 1;
			}
			
		}


		if (master.get_digital(R1) != lastKnowStateR1)
		{
			lastKnowStateR1 = master.get_digital(R1);
			if (master.get_digital(R1))
			{
				drive.setPose(0, 0, 0);
				drive.moveTo(0, 10, 0, 10000);
			}
			
		}

		if (master.get_digital(L1) != lastKnowStateL1)
		{
			lastKnowStateL1 = master.get_digital(L1);
			if (master.get_digital(L1))
			{
				drive.setPose(0, 0, 0);
				drive.moveTo(0, 0, 90, 10000);
			}
		}

		
		
	}
	
	
}
