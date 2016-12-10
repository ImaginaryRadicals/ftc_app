package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ashley on 11/10/2016.
 */

public class EncoderNavigation {


    public Hardware_iRads robot = null; // Robot Hardware class
    public ElapsedTime runtime = null; // Set by calling OpMode.
    public Telemetry telemetry = null; // Set by calling OpMode.


    // TimeStamps and MetaData
    // Absolute Position (setPosition, getPositionAge, getPositionDistance,
    private double absoluteX_mm = 0;
    private double absoluteY_mm = 0;
    private double absoluteHeading_deg = 0;
    private double absolutePositionConfidence = 0;
    private double absolutePositionTimeStamp = 0;
    private double absolutePositionDistance_mm = 0; // Distance traveled since absolute localization.

    private boolean isRelativeCalculated = false;
    private double  currentPositionConfidence = 0;

    private int lastEncoderPositionLeft = 0;
    private int lastEncoderPositionRight = 0;


    // Aggregated Inputs:
    private int stepsLeft = 0;
    private int stepsRight = 0;

    // Aggregated outputs: (should all be private with get() methods.)
    private double outputTimestamp = -1; // seconds
    //    public BodyRotation bodyRotation; // CCW, CW, or NONE
    private double radiusFromCenter_mm = 0; // Positive to robot's right side.
    private double arcDistance_mm = 0;
    private double deltaHeading_deg = 0; //Positive CCW
    // RobotX to right, RobotY in driving direction. (for this class, anyhow.)
    private double deltaRobotX_mm = 0;
    private double deltaRobotY_mm = 0;



    // Constructor
    public EncoderNavigation()
    {

    }

    public void initialize(Hardware_iRads robot, ElapsedTime runtime, Telemetry telemetry) {
        this.robot = robot;
        this.runtime = runtime;
        this.telemetry  = telemetry;
        // Initialize Encoder Positions.
        if (robot.hardwareEnabled) {
            lastEncoderPositionLeft = robot.leftDriveMotor.getCurrentPosition();
            lastEncoderPositionRight = robot.rightDriveMotor.getCurrentPosition();
        }
        else {
            lastEncoderPositionLeft = 0;
            lastEncoderPositionRight = 0;
        }
    }

    // Read encoder steps directly.
    public void setSteps() {
        if (robot.hardwareEnabled) {
            stepsLeft = robot.leftDriveMotor.getCurrentPosition() - lastEncoderPositionLeft;
            stepsRight = robot.rightDriveMotor.getCurrentPosition() - lastEncoderPositionRight;
        }
        else {
            stepsLeft = 0;
            stepsRight = 0;
        }
        setSteps(stepsLeft,stepsRight);
    }

    public void setSteps(int stepsLeft, int stepsRight) {
        isRelativeCalculated = false;
        this.stepsLeft = stepsLeft;
        this.stepsRight = stepsRight;
        this.outputTimestamp = runtime.time();

        // Keep encoder last position current.
        if (robot.hardwareEnabled) {
            lastEncoderPositionLeft = robot.leftDriveMotor.getCurrentPosition();
            lastEncoderPositionRight = robot.rightDriveMotor.getCurrentPosition();
        }

        // Crude estimate of drive distance.
        absolutePositionDistance_mm = ( Math.abs(stepsLeft) + Math.abs(stepsRight) ) /2 *
                robot.DRIVE_WHEEL_MM_PER_ROT / robot.DRIVE_WHEEL_STEPS_PER_ROT;

        calculateRelativeResults();
        calculateAbsoluteResults();
        calculatePositionConfidence();
    }


    private void calculatePositionConfidence() {
        // arbitrary weights set based on
        currentPositionConfidence = absolutePositionConfidence
                - 1.0 * getAbsoluteLocalizationAge()
                - .01 * absolutePositionDistance_mm;
    }


    private void calculateAbsoluteResults() {
        if (!isRelativeCalculated) calculateRelativeResults();

        // When Robot Heading = 0, its Y-axis is aligned with the game field's X-axis.
        // Note that this Robot Coordinate system definition is unique to this class.
        double PHASE_ROTATION_DEG = -90.0;

        // 2D Coordinate transformation.
        absoluteX_mm = absoluteX_mm
                + deltaRobotX_mm * Math.cos((absoluteHeading_deg + PHASE_ROTATION_DEG)* Math.PI/180.0)
                - deltaRobotY_mm * Math.sin((absoluteHeading_deg + PHASE_ROTATION_DEG)* Math.PI/180.0);

        absoluteY_mm = absoluteY_mm
                + deltaRobotX_mm * Math.sin((absoluteHeading_deg + PHASE_ROTATION_DEG)* Math.PI/180.0)
                + deltaRobotY_mm * Math.cos((absoluteHeading_deg + PHASE_ROTATION_DEG)* Math.PI/180.0);

        // Heading updated last because it is used for the transformation.
        absoluteHeading_deg = absoluteHeading_deg + deltaHeading_deg;
    }



    // setPosition, generally using the visionNavigation class.
    public void setPosition(double absoluteX_mm, double absoluteY_mm, double absoluteHeading_deg) {

        setPosition(absoluteX_mm, absoluteY_mm, absoluteHeading_deg,100);
    }

    public void setPosition(double absoluteX_mm,double absoluteY_mm, double absoluteHeading_deg,
                            double absolutePositionConfidence) {

        setPosition(absoluteX_mm,absoluteY_mm,absoluteHeading_deg,
                absolutePositionConfidence, runtime.time());
    }

    public void setPosition(double absoluteX_mm, double absoluteY_mm, double absoluteHeading_deg,
                            double absolutePositionConfidence, double absolutePositionTimeStamp) {

        isRelativeCalculated = false;
        this.absoluteX_mm = absoluteX_mm;
        this.absoluteY_mm = absoluteY_mm;
        this.absoluteHeading_deg = absoluteHeading_deg;
        this.absolutePositionConfidence = absolutePositionConfidence;
        this.absolutePositionTimeStamp = absolutePositionTimeStamp;

        this.absolutePositionDistance_mm = 0; // reset distance
        setSteps(0,0); // clear delta values and recalculate.
    }

    // Return age of encoder position in seconds.
    public double getEncoderStepAge() {
        return (runtime.time() - this.outputTimestamp);
    }

    public double getAbsoluteLocalizationAge() {
        return (runtime.time() - this.absolutePositionTimeStamp);
    }

    public void printResults() {
        telemetry.addData("Steps", String.format("Left: %d, Right: %d",stepsLeft,stepsRight) );
        telemetry.addData("arcRadius", String.format("%.1f mm ", radiusFromCenter_mm) );
        telemetry.addData("arcLength", String.format("%.1f mm ",arcDistance_mm) );
        telemetry.addData("deltaHeading", String.format("%.1f deg ",deltaHeading_deg) ); // positive CCW
        telemetry.addData("RobotX", String.format("%.1f mm",deltaRobotX_mm) );
        telemetry.addData("RobotY", String.format("%.1f mm",deltaRobotY_mm) );

    }


    private void calculateRelativeResults() {
    // Streamlined calculation.
    // Inputs: stepsLeft, stepsRight
    // Outputs: delta X, Y, Heading. (Radius/arc length optional?)

        int s1 = stepsLeft;
        int s2 = stepsRight;
        double rc = 0; // positive radius to right of robot center, in mm
        double w = robot.WHEELBASE_WIDTH_MM; // example, robot width mm.
        double arc = 0; // arc length of robot center mm.
        double DRIVE_MM_PER_STEP = robot.DRIVE_WHEEL_MM_PER_ROT / robot.DRIVE_WHEEL_STEPS_PER_ROT;

        double x = 0;
        double y = 0;
        double h = 0; // heading, deg

        // Calculate arc length
        arc = (s1 + s2)/2.0 * DRIVE_MM_PER_STEP;

        // Calculate arc radius, delta x,y,heading
        if (s1 == s2) {
            rc = Math.pow(10, 6); // Infinite radius.
            h = 0;
            x = 0;
            y = arc;
        } else if (s1 == -s2) {
            // Rotating in place
            rc = 0;  arc = 0; // not used.
            h = s2/(w/2) * DRIVE_MM_PER_STEP * (180/Math.PI);
            x = 0;
            y = 0;
        } else {
            rc = w * s1 /( s1 - s2) - w/2;
            h = -arc / rc * (180/Math.PI);
            x = rc * (1 - Math.cos( h * (Math.PI/180)));
            y = -rc * Math.sin( h * (Math.PI/180)); // negative radius.
        }


        // Outputs
        radiusFromCenter_mm = rc;
        arcDistance_mm = arc;
        deltaHeading_deg = h;
        deltaRobotX_mm = x;
        deltaRobotY_mm = y;

        isRelativeCalculated = true;

    } // calculateRelativeResults()





    // GETTERS

    public double getX() {
        return absoluteX_mm;
    }

    public double getY() {
        return absoluteY_mm;
    }

    public double getHeading() {
        return absoluteHeading_deg;
    }

    public double getAbsolutePositionDistance_mm() {
        return absolutePositionDistance_mm;
    }

    public double getConfidence() {
        return currentPositionConfidence;
    }

    public double getAge() {
        return getAbsoluteLocalizationAge();
    }
} // EncoderNavigation
