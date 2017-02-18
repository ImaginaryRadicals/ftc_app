package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ashley on 11/11/2016.
 * Class for collecting motor and servo commands before they are issued.
 */

public class NextMotorState {

    public Hardware_iRads robot = null;
    public Telemetry telemetry = null;


    // DcMotors:
    public double  leftDriveMotor      = 0;
    public double  rightDriveMotor     = 0;
    public double  leftLaunchMotor     = 0;
    public double  rightLaunchMotor    = 0;
    public double  liftMotor           = 0;
    // Servos:
    public double  launchTrigger       = Hardware_iRads.INITIAL_LAUNCHER_TRIGGER_POS;
    public double  leftFlipper         = Hardware_iRads.INITIAL_LEFT_FLIPPER_POS;
    public double  rightFlipper        = Hardware_iRads.INITIAL_RIGHT_FLIPPER_POS;

    /* Constructor */
    public NextMotorState() {}

    public void initialize(Hardware_iRads robot) {
        this.robot = robot;
    }

    public void initialize(Hardware_iRads robot, Telemetry telemetry){
        this.telemetry = telemetry;
        initialize(robot);
    }

    public void updateMotors() {

        robot.leftDriveMotor.setPower(leftDriveMotor);
        robot.rightDriveMotor.setPower(rightDriveMotor);
        robot.leftLaunchMotor.setPower(leftLaunchMotor);
        robot.rightLaunchMotor.setPower(rightLaunchMotor);
        robot.liftMotor.setPower(liftMotor);
        robot.launchTrigger.setPosition(launchTrigger);
        robot.leftFlipper.setPosition(leftFlipper);
        robot.rightFlipper.setPosition(rightFlipper);
    }

    public void print(){
        telemetry.addData("leftDriveMotor", leftDriveMotor);
        telemetry.addData("rightDriveMotor",rightDriveMotor);
        telemetry.addData("leftLaunchMotor", leftLaunchMotor);
        telemetry.addData("rightLaunchMotor", rightLaunchMotor);
        telemetry.addData("liftMotor", liftMotor);
        telemetry.addData("launchTrigger", launchTrigger);
        telemetry.addData("leftTrigger", leftFlipper);
        telemetry.addData("rightTrigger", rightFlipper);

    }

} // class
