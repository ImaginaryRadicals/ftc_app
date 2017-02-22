package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Utilities.*;

/**
 * Created by Stephen on 2/18/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ControlPID;
import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;

import java.util.Vector;

import java.util.concurrent.atomic.AtomicBoolean;

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

    Mutable<Double> startDelaySec = new Mutable<Double>(0.0);
    Mutable<Double> drivePowerLevel = new Mutable<Double>(0.5);
    Mutable<String> teamColor = new Mutable<String>("Red");
    Mutable<String> distanceFromGoal = new Mutable<String>("Near");
    Mutable<String> destination = new Mutable<String>("Center Vortex");
    //Boolean autoOpMode = new Boolean(true);
    Mutable<Boolean> launchWithPID = new Mutable<Boolean>(false);

    Vector<Double> left_launcher_pos = new Vector<>();
    Vector<Double> right_launcher_pos = new Vector<>();
    Vector<Double> time = new Vector<>();

    Mutable<Double> left_launcher_speed = new Mutable<>(0.0);
    Mutable<Double> left_launch_power = new Mutable<>(1.0);
    Mutable<Double> right_launcher_speed = new Mutable<>(0.0);
    Mutable<Double> right_launch_power = new Mutable<>(1.0);
    Mutable<Double> target_launch_speed = new Mutable<>(0.0);

    private ControlPID left_pid; // instantiated in constructor
    private ControlPID right_pid; // instantiated in constructor

    double turnDistance = 0.0;

    iRadsAutoOp()
    {
        left_pid = new ControlPID(left_launcher_speed, target_launch_speed, left_launch_power);
        left_pid.setGains(0.5, 0.1, 0.01);
        right_pid = new ControlPID(right_launcher_speed, target_launch_speed, right_launch_power);
        right_pid.setGains(0.5, 0.1, 0.01);
    }

    @Override
    public void runOpMode() {
        initialize();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("startDelaySec: ", startDelaySec.get().toString());
        telemetry.addData("launchWithPID: ", launchWithPID.get().toString());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);

        //sleep for the total length of the driver-input delay
        //it is possible that it might sleep so long that we go past 30 sec in the autonomous

        sleep((int)(1000.0 * startDelaySec.get()));

        setLaunchPower(1);

        sleep(3500); //stop for a little while longer so that the launch motors can finish getting up to speed

        //launch the two particles and pull power from the launch motors and close the flippers
        if (launchWithPID.get())
            launchBallsPID();
        else
            launchBalls();

        if (destination.get() == "Center Vortex") {

            driveToCenterVortex();

        } else if (destination.get() == "Corner Vortex") {

            driveToCornerVortex();
            
        }

    }

    public void initialize() {

        telemetry.addData("Warning:", " You forgot to initialize gamepad1.");

        robot.init(hardwareMap);
        resetEncoders();

        interactive = new InteractiveInit(telemetry, gamepad1, this);

        interactive.addDouble(startDelaySec, "startDelaySec", 0.0, 5.0, 10.0);
        interactive.addDouble(drivePowerLevel, "drivePowerLevel", 0.25, 0.5, 1.0);
        interactive.addString(teamColor, "teamColor", "Red", "Blue");
        interactive.addString(distanceFromGoal, "distanceFromGoal", "Near", "Far");
        //interactive.addBoolean(autoOpMode, "autoOpMode", false, true);
        interactive.addString(destination, "destination", "Center Vortex", "Corner Vortex");
        interactive.addBoolean(launchWithPID, "launchWithPID", true, false);

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
        robot.rightDriveMotor.setPower(drivePowerLevel.get());
        robot.leftDriveMotor.setPower(drivePowerLevel.get());

        robot.leftDriveMotor.setTargetPosition(
                mmToTicks(distance_mm)
        );
        robot.rightDriveMotor.setTargetPosition(
                mmToTicks(distance_mm)
        );
    }

    void turn(double theta) {

        resetEncoders();

        // positive turns right, negative turns left.  Units are degrees
        turnDistance = robot.WHEELBASE_WIDTH_MM * 3.14159 * theta / 360;
        turnDistance = mmToTicks(turnDistance);
        turnDistance *= 1.5;
        robot.leftDriveMotor.setTargetPosition((int) -turnDistance);
        robot.rightDriveMotor.setTargetPosition((int) turnDistance);

        robot.rightDriveMotor.setPower(1);
        robot.leftDriveMotor.setPower(1);

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

    private void runLaunchMotors(AtomicBoolean run_launch_motors)
    {
        VectorMath vector_math = new VectorMath();

        while (run_launch_motors.get())
        {
            left_launcher_pos.add((double)robot.leftDriveMotor.getCurrentPosition());
            right_launcher_pos.add((double)robot.rightDriveMotor.getCurrentPosition());
            time.add(runtime.seconds());

            try {
                telemetry.addData("Error: ", "derivative update");
            }
            catch (RuntimeException e)
            {
                left_launcher_speed.set(vector_math.derivative(time, left_launcher_pos));
                right_launcher_speed.set(vector_math.derivative(time, right_launcher_pos));
            }

            try {
                left_pid.update(runtime.time());
                right_pid.update(runtime.time());
            }
            catch (RuntimeException e)
            {
                telemetry.addData("Error: ", "pid update");
            }
        }
    }

    public void launchBallsPID()
    {
        final AtomicBoolean run_launch_motors = new AtomicBoolean(true);

        if (distanceFromGoal.get() == "Near") {
            target_launch_speed.set(1050.0);
        } else
        {
            target_launch_speed.set(1150.0);
        }

        // Open the flippers so we can shoot
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_OPEN);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_OPEN);
        sleep(500);

        // launch once
        new Thread(new Runnable() {
            public void run(){
                runLaunchMotors(run_launch_motors);
            }
        }).start();

        sleep(5000); // give the motors time to hit the target speed

        telemetry.addData("left_launcher_speed: ", left_launcher_speed.toString());
        telemetry.addData("right_launcher_speed: ", right_launcher_speed.toString());
        telemetry.addData("target_launch_speed: ", target_launch_speed.toString());
        telemetry.update();

        // launch once
        robot.launchTrigger.setPosition(robot.ELEVATED_LAUNCHER_TRIGGER_POS);
        sleep(500);
        robot.launchTrigger.setPosition(robot.INITIAL_LAUNCHER_TRIGGER_POS);
        sleep(500);

        // Close the flippers to prevent the second particle from escaping
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);
        sleep(5000);

        telemetry.addData("left_launcher_speed: ", left_launcher_speed.toString());
        telemetry.addData("right_launcher_speed: ", right_launcher_speed.toString());
        telemetry.addData("target_launch_speed: ", target_launch_speed.toString());
        telemetry.update();

        // launch a second time
        robot.launchTrigger.setPosition(robot.ELEVATED_LAUNCHER_TRIGGER_POS);
        sleep(500);
        robot.launchTrigger.setPosition(robot.INITIAL_LAUNCHER_TRIGGER_POS);

        run_launch_motors.set(false);
    }

    void launchBalls(){

        //set proper max speeds based on InteractiveInit input

        if (distanceFromGoal.get() == "Near") {
            robot.leftLaunchMotor.setMaxSpeed(1000);
            robot.rightLaunchMotor.setMaxSpeed(1000);
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

    void driveToCenterVortex() {

        resetEncoders();

        setLaunchPower(0);
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);

        // drive forward again to knock off the cap ball and park on its platform
        if (distanceFromGoal.get() == "Near") {

            driveForward(1500);

        } else {

            driveForward(2000);

        }

        sleepWhileDriving();

    }

    void driveToCornerVortex() {

        resetEncoders();

        //cease launching
        setLaunchPower(0);
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);

        // get off the wall
        if (distanceFromGoal.get() == "Far") {

            driveForward(800);

            sleepWhileDriving();

            // orient towards corner vortex
            if (teamColor.get() == "Red")       turn(49.66);
            else if (teamColor.get() == "Blue") turn(-49.66);

        } else {

            driveForward(610);

            sleepWhileDriving();

            // orient towards corner vortex
            if (teamColor.get() == "Red")       turn(90);
            else if (teamColor.get() == "Blue") turn(-90);

        }

        sleepWhileDriving();

        //drive to corner vortex
        if (distanceFromGoal.get() == "Far") {

            driveForward(1725);

        } else {

            driveForward(950);

        }

        sleepWhileDriving();

        //turn into corner vortex
        if (teamColor.get() == "Red") turn(45);
        else if (teamColor.get() == "Blue") turn(-45);

        sleepWhileDriving();

        //drive up the ramp
        driveForward(500);

        sleepWhileDriving();

    }

    void sleepWhileDriving () {

        while (robot.leftDriveMotor.isBusy() && robot.rightDriveMotor.isBusy()){
            sleep(50);
        }

    }
}

