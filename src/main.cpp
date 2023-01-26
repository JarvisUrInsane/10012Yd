#include "main.h"
#include <sstream>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}
Controller controller;
Motor intake(14);
pros::Motor flywheel(-15);
IMU imu(17);

ControllerButton intakeIn(ControllerDigital::R2);
ControllerButton intakeOut(ControllerDigital::R1);
ControllerButton flywheelSpin(ControllerDigital::L1);
ControllerButton switchMode(ControllerDigital::X);

auto drive = ChassisControllerBuilder()
                 .withMotors({-11, -12, -13}, {18, 19, 20})
                 // Green gearset, 4 in wheel diam, 11.5 in wheel track
                 .withDimensions({AbstractMotor::gearset::blue, (60.0 / 36.0)},
                                 {{3.25_in, 12_in}, imev5BlueTPR})
                 .withGains({0.0009, 0, 0.000001},  // Distance controller gains
                            {0.0005, 0.002, 0.0001} // Turn controller gains
                            )
                 //  .withSensors(12, 19, ADIEncoder{'A', 'B'})
                 //  .withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
                 //  .buildOdometry();
                 .build();
std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
            1.0, // Maximum linear velocity of the Chassis in m/s
            2.0, // Maximum linear acceleration of the Chassis in m/s/s
            10.0 // Maximum linear jerk of the Chassis in m/s/s/s
        })
        .withOutput(drive)
        .buildMotionProfileController();
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  drive->getModel()->setBrakeMode(AbstractMotor::brakeMode::brake);
  intake.setBrakeMode(AbstractMotor::brakeMode::brake);
  flywheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.moveVelocity(600);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
const int intakeVoltage = 10000;
struct RobotControls {
private:
  pros::Task robotControlsDaemon;

public:
  bool flywheelState, intakeState, indexerState, moveSlowState;
  RobotControls()
      : flywheelState(false), intakeState(false), indexerState(false),
        moveSlowState(false), robotControlsDaemon([=] {
          while (69) {
            if (flywheelState) {
              flywheel.move_voltage(12000);
            } else {
              flywheel.move_voltage(0);
            }
            if (intakeState) {
              intake.moveVoltage(intakeVoltage);
            } else if (indexerState) {
              intake.moveVoltage(-intakeVoltage / 2);
            } else {
              intake.moveVoltage(0);
            }
            if (moveSlowState) {
              drive->setMaxVelocity(150);
            } else {
              drive->setMaxVelocity(600);
            }
            pros::delay(20);
          }
        }) {}
  void resume() { robotControlsDaemon.resume(); }
  void stop() { robotControlsDaemon.suspend(); }
} robotControls;

void autonomous() {
  robotControls.intakeState = true;
  robotControls.moveSlowState = true;
  drive->moveDistanceAsync(60_in);
  drive->waitUntilSettled();
  pros::delay(1000);
  robotControls.intakeState = false;
  robotControls.flywheelState = true;
  pros::delay(5000);
  robotControls.indexerState = true;
  pros::delay(3000);
  robotControls.moveSlowState = false;
  drive->moveDistanceAsync(-40_in);
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
void opcontrol() {
  bool flywheelOn = false;
  bool arcadeDrive = true;
  while (true) {
    drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                              controller.getAnalog(ControllerAnalog::rightX));
    if (intakeIn.isPressed()) {
      intake.moveVoltage(intakeVoltage);
    } else if (intakeOut.isPressed()) {
      intake.moveVoltage(-intakeVoltage);
    } else {
      intake.moveVoltage(0);
    }

    if (flywheelSpin.changedToReleased()) {
      flywheelOn = !flywheelOn;
    }
    if(flywheelOn){
      flywheel.move_voltage(12000);
    } else{
      flywheel.move_voltage(0);
    }
    if (switchMode.isPressed()) {
      arcadeDrive = !arcadeDrive;
    }
    if (arcadeDrive) {
      drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                                controller.getAnalog(ControllerAnalog::rightX));
    } else {
      drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                              controller.getAnalog(ControllerAnalog::rightY));
    }
    pros::screen::set_pen(COLOR_BLUE);
    pros::screen::print(pros::E_TEXT_LARGE, 3, std::to_string(flywheel.get_voltage()).c_str());

    pros::delay(10);
  }
  
}
