package org.firstinspires.ftc.teamcode;

/**
 * Created by Stephen on 2/18/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;

/**
 * This autonomous opmode will drive forward a specified distance, shoot 2 particles, and drive the
 * rest of the way to knock off the cap ball.  Note: the launch trigger will pump 3x.  This is so
 * that if one of the balls isn't launched properly, we can recover.
 * */

@Autonomous(name="iRadsAutoOp", group="iRads")
//@Disabled
public class iRadsAutoOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    Hardware_iRads robot = new Hardware_iRads();
    private InteractiveInit interactive; // Initialized in "initialize()" method.

    Double startDelaySec = new Double(0.0);
    Double drivePowerLevel = new Double(0.5);
    String teamColor = new String("Red");
    String distanceFromGoal = new String("Near");
    Boolean autoOpMode = new Boolean(true);

    @Override
    public void runOpMode() {
        initialize();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);

        //sleep for the total length of the driver-input delay
        //it is possible that it might sleep so long that we go past 30 sec in the autonomous

        sleep(startDelaySec.intValue());

        setLaunchPower(1);

        sleep(3500); //stop for a little while longer so that the launch motors can finish getting up to speed

        //launch the two particles and pull power from the launch motors and close the flippers
        launchBalls();
        setLaunchPower(0);
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);

        // drive forward again to knock off the cap ball and park on its platform
        if (distanceFromGoal == "Near")
            driveForward(1500);
        else
            driveForward(2000);

        //tells the robot to keep doing what it is doing until the drive motors are finished
        while (robot.leftDriveMotor.isBusy()){
            sleep(50);
        }
    }

    public void initialize() {

        if (gamepad1 == null)
            throw new RuntimeException("ERROR: You forgot to initialize gamepad1.");

        robot.init(hardwareMap);
        resetEncoders();

        interactive = new InteractiveInit(telemetry, gamepad1, this);

        interactive.addDouble(startDelaySec, "startDelaySec", 0.0, 5.0, 10.0);
        interactive.addDouble(drivePowerLevel, "drivePowerLevel", 0.25, 0.5, 1.0);
        interactive.addString(teamColor, "teamColor", "Red", "Blue");
        interactive.addString(distanceFromGoal, "distanceFromGoal", "Near", "Far");
        interactive.addBoolean(autoOpMode, "autoOpMode", false, true);

        interactive.menuInputLoop();
    }

    int mmToTicks(double driveDistance_mm) {
        driveDistance_mm *= .65;
        /*Don't ask why.  In testing, the robot drove 1.5 times the distance
        intended, and the reciprocal of 1.5 is roughly .65.  This is probably due to lack of
        traction, in which case this factor of 6.5 will prove roughly accurate.  After multiplying
        by this factor, the robot goes to its desired position +- 1cm.*/

        //convert mm to ticks
        double driveDistance_ticks = driveDistance_mm  * robot.DRIVE_WHEEL_STEPS_PER_ROT / robot.DRIVE_WHEEL_MM_PER_ROT;
        return (int) driveDistance_ticks;
    }

    void driveForward(double distance_mm){

        resetEncoders();

        /*It is important that the power is set before the target position.  If it is not, the robot
        * will attempt to reach its target position at a speed of zero, which is not that efficient.
        * */
        robot.rightDriveMotor.setPower(drivePowerLevel);
        robot.leftDriveMotor.setPower(drivePowerLevel);

        robot.leftDriveMotor.setTargetPosition(
                mmToTicks(distance_mm)
        );
        robot.rightDriveMotor.setTargetPosition(
                mmToTicks(distance_mm)
        );
    }

    void setLaunchPower(double launchPower) {
        robot.rightLaunchMotor.setPower(launchPower);
        robot.leftLaunchMotor.setPower(launchPower);
    }

    void resetEncoders() {
        robot.leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    void launchBalls(){

        //set proper max speeds based on InteractiveInit input

        if (distanceFromGoal == "Near") {
            robot.leftLaunchMotor.setMaxSpeed(1050);
            robot.rightLaunchMotor.setMaxSpeed(1050);
        } else
        {
            robot.leftLaunchMotor.setMaxSpeed(1150);
            robot.rightLaunchMotor.setMaxSpeed(1150);
        }

        //Open the flippers so we can shoot
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_OPEN);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_OPEN);
        sleep(500);

        //launch once
        robot.launchTrigger.setPosition(robot.ELEVATED_LAUNCHER_TRIGGER_POS);
        sleep(500);
        robot.launchTrigger.setPosition(robot.INITIAL_LAUNCHER_TRIGGER_POS);
        sleep(500);

        // Close the flippers to prevent the second particle from escaping
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);
        sleep(5000);

        //launch a second time
        robot.launchTrigger.setPosition(robot.ELEVATED_LAUNCHER_TRIGGER_POS);
        sleep(500);
        robot.launchTrigger.setPosition(robot.INITIAL_LAUNCHER_TRIGGER_POS);
    }

}

