package org.firstinspires.ftc.teamcode;

/**
 * Created by Ashley on 11/11/2016.
 * Class for collecting motor and servo commands before they are issued.
 */

import static org.firstinspires.ftc.teamcode.Hardware_iRads.INITIAL_LAUNCHER_TRIGGER_POS;

public class NextMotorState {

    public Hardware_iRads robot = null;

    // DcMotors:
    public double  leftDriveMotor      = 0;
    public double  rightDriveMotor     = 0;
    public double  leftLaunchMotor     = 0;
    public double  rightLaunchMotor    = 0;
    public double  liftMotor           = 0;
    // Servos:
    public double  launchTrigger       = Hardware_iRads.INITIAL_LAUNCHER_TRIGGER_POS;

    /* Constructor */
    public NextMotorState() {}

    public void initialize(Hardware_iRads robot) {
        this.robot = robot;
    }

    public void updateMotors() {

        robot.leftDriveMotor.setPower(leftDriveMotor);
        robot.rightDriveMotor.setPower(rightDriveMotor);
        robot.leftLaunchMotor.setPower(leftLaunchMotor);
        robot.rightLaunchMotor.setPower(rightLaunchMotor);
        robot.liftMotor.setPower(liftMotor);
        robot.launchTrigger.setPosition(launchTrigger);

    }

} // class
