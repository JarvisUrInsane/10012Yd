#include "main.h"

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
Motor flywheel(16);
IMU imu(17);

ControllerButton intakeIn(ControllerDigital::R2);
ControllerButton intakeOut(ControllerDigital::R1);
ControllerButton flywheelSpin(ControllerDigital::L1);

auto drive = ChassisControllerBuilder()
                 .withMotors({-11, -12, -13}, {18, 19, 20})
                 // Green gearset, 4 in wheel diam, 11.5 in wheel track
                 .withDimensions({AbstractMotor::gearset::blue, (36.0 / 60.0)},
                                 {{3.25_in, 12_in}, imev5BlueTPR})
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
  flywheel.setBrakeMode(AbstractMotor::brakeMode::coast);
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
void autonomous() {
  imu.calibrate();
  imu.reset();
  drive->moveDistance(61_cm);
  drive->waitUntilSettled();
  drive->turnAngle(180_deg);
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
  while (true) {
    drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                              controller.getAnalog(ControllerAnalog::rightX));
    if (intakeIn.isPressed()) {
      intake.moveVoltage(12000);
    } else if (intakeOut.isPressed()) {
      intake.moveVoltage(-12000);
    } else {
      intake.moveVoltage(0);
    }

    if (flywheelSpin.changedToReleased()) {
      flywheelOn = !flywheelOn;
    }
    flywheel.moveVoltage(flywheelOn?12000:0);

    pros::delay(10);
  }
