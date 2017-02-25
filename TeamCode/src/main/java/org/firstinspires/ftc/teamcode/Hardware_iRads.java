package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Utilities.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Vector;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is the iRads robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * DcMotors:
 * Motor channel:  Left  drive motor:        "leftDriveMotor"
 * Motor channel:  Right drive motor:        "rightDriveMotor"
 * Motor channel:  Left  launch motor:       "leftLaunchMotor"
 * Motor channel:  Right launch motor:       "rightLaunchMotor"
 * Motor channel:  Cap ball lift motor:      "liftMotor"
 * Servos:
 * Servo channel:  Servo to launch ball:     "launchTrigger"
 */
public class Hardware_iRads
{
    /* Public OpMode members. */
    // DcMotors:
    public DcMotor  leftDriveMotor      = null;
    public DcMotor  rightDriveMotor     = null;
    public DcMotor  leftLaunchMotor     = null;
    public DcMotor  rightLaunchMotor    = null;
    public DcMotor  liftMotor           = null;
    // Servos:
    public Servo    launchTrigger       = null;
    public Servo    leftFlipper         = null;
    public Servo    rightFlipper        = null;

    public static boolean hardwareEnabled = true; // set false if hardware errors are encountered

    public static final double MID_SERVO            =  0.5 ;
    public static final double INITIAL_LAUNCHER_TRIGGER_POS = 0.94;
    public static final double ELEVATED_LAUNCHER_TRIGGER_POS = 0.7;

    // public doubles for the left and right flipper
    public static final double INITIAL_LEFT_FLIPPER_POS = 0.66;
    public static final double INITIAL_RIGHT_FLIPPER_POS = 0.40;
    public static final double LEFT_FLIPPER_OPEN = 0.0;
    public static final double LEFT_FLIPPER_CLOSED = 0.66;
    public static final double RIGHT_FLIPPER_OPEN = 0.0;
    public static final double RIGHT_FLIPPER_CLOSED = 0.66;

    public static final double LAUNCH_WHEEL_DIAMETER_INCHES =  4;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES  =  4;

    public static final double MM_PER_IN = 25.4f;
    public static final double WHEELBASE_WIDTH_IN = 13;
    public static final double WHEELBASE_WIDTH_MM  = WHEELBASE_WIDTH_IN  * MM_PER_IN;
    public static final double DRIVE_WHEEL_RADIUS_MM = DRIVE_WHEEL_DIAMETER_INCHES /2.0 * MM_PER_IN;
    public static final double DRIVE_WHEEL_MM_PER_ROT = DRIVE_WHEEL_RADIUS_MM * 2 *  Math.PI;

    public static final int DRIVE_WHEEL_STEPS_PER_ROT     = 28*60;
    public static final double LAUNCH_WHEEL_STEPS_PER_ROT    =  28*3.7 ; // Ticks per rotation
    public static final int LIFT_STEPS_PER_ROT            =  28*60; // Ticks per rotation

    public static final int MAX_DRIVE_SPEED_TPS     =  1680 ; // Ticks per second
    public static final int LIFT_MAX_SPEED_TPS      =  1680 ; // Ticks per second
    public static final int LAUNCH_WHEEL_RPM        =  1100;   // Max RPM
    public static final int MAX_LAUNCH_SPEED_TPS    =   1150; // Ticks per second
//    public static final int MAX_LAUNCH_SPEED_TPS    =   (int) LAUNCH_WHEEL_STEPS_PER_ROT * LAUNCH_WHEEL_RPM / 60 ; // Ticks per second

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private Vector pastPeriods  = new Vector();


    /* Constructor */
    public Hardware_iRads(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDriveMotor    = hwMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor   = hwMap.dcMotor.get("rightDriveMotor");
        leftLaunchMotor   = hwMap.dcMotor.get("leftLaunchMotor");
        rightLaunchMotor  = hwMap.dcMotor.get("rightLaunchMotor");
        liftMotor         = hwMap.dcMotor.get("liftMotor");
        // Set Motor Direction
        leftDriveMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftLaunchMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightLaunchMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
        leftLaunchMotor.setPower(0);
        rightLaunchMotor.setPower(0);
        liftMotor.setPower(0);


        // Set all motors to run with encoders.
        // Use RUN_WITHOUT_ENCODERS if encoders are NOT installed.
        // Use RUN_USING_ENCODERS if encoders ARE installed.
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set ENCODER mode max speed (in Ticks per second)
        leftDriveMotor.setMaxSpeed(MAX_DRIVE_SPEED_TPS);
        rightDriveMotor.setMaxSpeed(MAX_DRIVE_SPEED_TPS);
        leftLaunchMotor.setMaxSpeed(MAX_LAUNCH_SPEED_TPS);
        rightLaunchMotor.setMaxSpeed(MAX_LAUNCH_SPEED_TPS);
        liftMotor.setMaxSpeed(LIFT_MAX_SPEED_TPS);

        // Allow launch wheels to coast.
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




        // Define and initialize ALL installed servos.
        launchTrigger = hwMap.servo.get("launchTrigger");
        launchTrigger.setPosition(INITIAL_LAUNCHER_TRIGGER_POS);

        leftFlipper = hwMap.servo.get("leftFlipper");
        leftFlipper.setPosition(INITIAL_LEFT_FLIPPER_POS);

        rightFlipper = hwMap.servo.get("rightFlipper");
        rightFlipper.setPosition(INITIAL_RIGHT_FLIPPER_POS);
        rightFlipper.setDirection(Servo.Direction.REVERSE);


    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public double periodSec(){

        pastPeriods.add(period.time());
        period.reset();
        if (pastPeriods.size()>= 30) {
            pastPeriods.remove(0);
        }
        return VectorMath.average(pastPeriods);
    }
}

