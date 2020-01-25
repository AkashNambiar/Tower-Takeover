#include "main.h"
using namespace okapi;
struct PID
{
	float kP;
	float kI;
	float kD;
	float integral;
	float derivative;
	float error;
	float previous_error;
	float speed;
	float target;
	float sensor;
};
typedef struct PID pid;
pid LT;
pid TL;
//others

pros::ADIPotentiometer trayPot('H');
pros::ADIPotentiometer rollerLiftPot('F');
//pros::ADIEncoder trackingWheel('B', 'C');
//Inertia sensor: port 13
okapi::Controller controller;

okapi::Motor trayLift(-16);
okapi::Motor rollerLift(9);

okapi::MotorGroup rollers({-5, 8});
okapi::Motor rollerOne(-5);
okapi::Motor rollerTwo(8);

int lcdCounter = 1;
int buttonCount = 0;
bool isPressed = false;

double slowTraySpeed = 27.5;
double fastTraySpeed = 200;

bool holdTray = false;
int trayPosition = 400;
bool holdRollerLift = false;
int rollerLiftToggle = 0;
int rollerLiftPosition;

double slowMoveKP = 0.0005; //0.001
double fastMoveKP = 0.002;
int holdToggle = 0;
auto chassis = okapi::ChassisControllerFactory::create({20, 19}, {-11, -3}, okapi::AbstractMotor::gearset::green, {4.125, 10});

void rollerLiftToggleTask(void *param);
void trayPIDTask(void *param);
void rollerLiftPIDTask(void *param);
void trayToggleTask(void *param);

void opcontrol()
{

	pros::Task rollerLiftToggleTaskHandle(rollerLiftToggleTask);
	pros::Task trayPIDTaskHandle(trayPIDTask);
	pros::Task rollerLiftPIDTaskHandle(rollerLiftPIDTask);
	pros::Task trayToggleTaskHandle(trayToggleTask);
	rollerLiftPosition = 400;
	//holdRollerLift = 0;

	while (true)
	{
		
		//std::cout << "\n"
		//		  << rollerLift.getPosition() << " " << LT.target<<" "<<rollerLiftToggle;
		chassis.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		rollers.moveVelocity(200 * controller.getDigital(ControllerDigital::L1) - 200 * controller.getDigital(ControllerDigital::Y) - 50 * controller.getDigital(ControllerDigital::L2));
		
		pros::delay(20);
	}
}

void trayLiftPID(double value)
{
	TL.target = value;
	TL.integral = 0;
	TL.sensor = trayPot.get_value();
	TL.error = TL.target - TL.sensor;
	int timer = 0;

	while (true)						 //check GIT
	{									 //or while(timer < 50){
		TL.kP = 0.2;					 //need tuning
		TL.kD = 0.1;					 //need tuning
		TL.kI = 0;						 //need tuning
		TL.sensor = trayPot.get_value(); // = sensor.getValue || post setup
		TL.error = TL.target - TL.sensor;
		TL.derivative = TL.error - TL.previous_error;
		TL.integral += TL.error;
		TL.previous_error = TL.error;
		TL.speed = (TL.kP * TL.error + TL.kD * TL.derivative + TL.kI * TL.integral);
		trayLift.moveVelocity(TL.speed);
		//fill
		timer++;
		pros::delay(20);
		//std::cout << "\nPot Value:" << trayPot.get_value();
	}
}

void rollerLiftPID(double degrees)
{
	LT.target = degrees;
	LT.integral = 0;
	LT.sensor = rollerLift.getPosition();
	LT.error = LT.target - LT.sensor;
	int timer = 0;

	while (true)//(abs(LT.error) >= 40)
	{										  //or while(timer < 50){
		LT.kP = 0.15;						  //need tuning
		LT.kD = 0.1;						  //need tuning
		LT.kI = 0;							  //need tuning
		LT.sensor = rollerLift.getPosition(); // = sensor.getValue || post setup
		LT.error = LT.target - LT.sensor;
		LT.derivative = LT.error - LT.previous_error;
		LT.integral += LT.error;
		LT.speed = (LT.kP * LT.error + LT.kD * LT.derivative + LT.kI * LT.integral);
		rollerLift.moveVelocity(LT.speed);
		//fill
		timer += 20;
		std::cout << "\n"
				  << rollerLift.getPosition();
		pros::delay(20);
	}
}

void movePID(double distanceL, double distanceR, double speedkP, int ms)
{
	double targetL = distanceL * 360 / (2 * 3.1415 * (4.125 / 2));
	double targetR = distanceR * 360 / (2 * 3.1415 * (4.125 / 2));
	auto drivePIDL = okapi::IterativeControllerFactory::posPID(speedkP, 0.001, 0.0015); //= data
	auto drivePIDR = okapi::IterativeControllerFactory::posPID(speedkP, 0.001, 0.0015);
	chassis.resetSensors();

	int timer = 0;
	double errorL;
	double errorR;
	double powerL;
	double powerR;

	while (timer < ms)
	{
		errorL = targetL - chassis.getSensorVals()[0];
		errorR = targetR - chassis.getSensorVals()[1];
		powerL = drivePIDL.step(errorL);
		powerR = drivePIDR.step(errorR);
		chassis.tank(-powerL, -powerR);

		pros::delay(10);
		timer += 10;
	}

	chassis.tank(0, 0);
}

void trayToggleTask(void *)
{
	while (true)
	{
		if (controller[ControllerDigital::right].changedToPressed())
		{
			holdTray = !holdTray;
			pros::delay(100);
		}

		if (holdTray)
		{
			trayPosition = 400;
		}
		else
		{
			trayLift.moveVelocity(slowTraySpeed * controller.getDigital(ControllerDigital::R1) +
								  fastTraySpeed * controller.getDigital(ControllerDigital::left) - fastTraySpeed * controller.getDigital(ControllerDigital::R2));
		}
		pros::delay(25);
	}
}

void trayPIDTask(void *)
{
	while (true)
	{
		//std::cout << "\nPot Value:" << holdTray;
		TL.target = trayPosition;
		TL.integral = 0;
		TL.sensor = trayLift.get_position();
		TL.error = TL.target - TL.sensor;
		int timer = 0;

		while (holdTray)						 //(abs(TL.error) >= 40)
		{										 //or while(timer < 50){
			TL.kP = 0.4;						 //need tuning
			TL.kD = 0;							 //need tuning
			TL.kI = 0;							 //need tuning
			TL.sensor = trayLift.get_position(); // = sensor.getValue || post setup
			TL.error = TL.target - TL.sensor;
			TL.derivative = TL.error - TL.previous_error;
			TL.integral += TL.error;
			TL.previous_error = TL.error;
			TL.speed = (TL.kP * TL.error + TL.kD * TL.derivative + TL.kI * TL.integral);
			trayLift.moveVelocity(TL.speed);
			//fill
			timer += 20;
			//std::cout << "\nPot Value:" << trayLift.get_position();
			pros::delay(20);
		}
	}
}

void rollerLiftToggleTask(void *)
{
	while (true)
	{
		if (controller[ControllerDigital::A].changedToPressed())
		{
			//std::cout << "Button A Pressed\n";
			rollerLiftToggle++;
			if (rollerLiftToggle == 3)
			{
				rollerLiftToggle = 0;
				holdRollerLift = false;
			}
			std::cout << "\n" << "button pressed";
			pros::delay(200);
		}

		if (rollerLiftToggle != 0){
			holdTray = true;
		}

		if (rollerLiftToggle == 1)
		{
			rollerLiftPosition = 400;
			holdRollerLift = true;
			std::cout << "\n" << "1 "<<rollerLiftPosition;
		}
		else if (rollerLiftToggle == 2)
		{
			rollerLiftPosition = 560;
			holdRollerLift = true;
			std::cout << "\n" << rollerLiftPosition;
		}
		else
		{
			rollerLift.controllerSet(controller.getDigital(ControllerDigital::X) - controller.getDigital(ControllerDigital::B));
		}

		pros::delay(25);
	}
}

void rollerLiftPIDTask(void *)
{
	while (true)
	{
		//LT.target = rollerLiftPosition;
		LT.integral = 0;
		LT.sensor = rollerLift.getPosition();
		LT.error = LT.target - LT.sensor;
		int timer = 0;

		while (holdRollerLift)
		{										  //or while(timer < 50){
			LT.kP = 0.4;						  //need tuning
			LT.kD = 0.2;						  //need tuning
			LT.kI = 0;							  //need tuning
			LT.sensor = rollerLift.getPosition(); // = sensor.getValue || post setup
			LT.error = rollerLiftPosition - LT.sensor;
			LT.derivative = LT.error - LT.previous_error;
			LT.integral += LT.error;
			LT.speed = (LT.kP * LT.error + LT.kD * LT.derivative + LT.kI * LT.integral);
			rollerLift.moveVelocity(LT.speed);
			//fill
			timer += 20;
			
			pros::delay(20);
		}
	}
}
///dev/cu.usbmodem14101


//Autonomous

void red()
{
	movePID(30, 30, fastMoveKP, 1500);
}

void redBig()
{
}

void push()
{
	movePID(30, 30, fastMoveKP, 1500);
	movePID(-30, -30, fastMoveKP, 1500);
}

void blue()
{
}

void autonomous()
{
	switch (lcdCounter)
	{
	case 0:
		break;
	case 1:
		red();
		break;
	case 2:
		push();
		break;
	case 3:
		blue();
		break;
	case 4:
		redBig();
		break;
	}
}

bool selected = true; //TODO: false

void left_button()
{
	if (!selected)
	{
		lcdCounter--;
		if (lcdCounter < 0)
		{
			lcdCounter = 0;
		}
	}
}
void center_button()
{
	if (!selected)
	{
		selected = true;
	}
}
void right_button()
{
	if (!selected)
	{
		lcdCounter++;
		if (lcdCounter > 4)
		{
			lcdCounter = 4;
		}
	}
}
std::string convert(int arg)
{
	switch (arg)
	{
	case 0:
		return "No Auton";
	case 1:
		return "Red";
	case 2:
		return "Push";
	case 3:
		return "Blue";
	case 4:
		return "Red Big";
	default:
		return "No Auton";
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(left_button);
	pros::lcd::register_btn1_cb(center_button);
	pros::lcd::register_btn2_cb(right_button);
	//pros::lcd::print(0, "Test Temperature", pros::trayLift::get_temperature());

	//intakeLS.calibrate();
	//rollers.calibrate();
	//indexerLS.calibrate();
	//hoodLS.calibrate();

	while (!selected)
	{
		pros::lcd::set_text(0, convert(lcdCounter));
		pros::delay(20);
	}

	pros::lcd::set_text(0, convert(lcdCounter) + " (SELECTED)");

	//pros::Task trayTaskHandle(trayTask);
	//pros::Task armTaskHandle(armTask);
	//pros::Task trayLiftTaskHandle(trayLiftTask);
}

//void disabled() {}

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

//void competitionitialize() {}
