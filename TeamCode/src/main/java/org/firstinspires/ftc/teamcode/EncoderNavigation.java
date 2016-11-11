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

    public enum RotationMode {SAME_DIRECTION, OPPOSITE_DIRECTION};
    public enum RotationCenter {LEFT_SIDE, RIGHT_SIDE, CENTERED};
    public enum RobotDirection {FORWARD, REVERSE}; //
//    public enum BodyRotation {CCW, CW, NO);

    // Aggregated Inputs:
    private int stepsLeft = 0;
    private int stepsRight = 0;

    // Aggregated outputs: (should all be private with get() methods.)
    private double outputTimestamp = -1; // seconds
    private RotationMode rotationMode; // Wheels spining in same or opposite directions?
    private RotationCenter rotationCenter; // which side is the rotation point on?
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

    }


    public void setSteps(int stepsLeft, int stepsRight) {
        this.stepsLeft = stepsLeft;
        this.stepsRight = stepsRight;
        //this.outputTimestamp = runtime.time();
        this.outputTimestamp = runtime.time();
        calculateResults();
    }

    // Return age of encoder position in seconds.
    public double getAge() {
        return (runtime.time() - this.outputTimestamp);
    }

    public void printResults() {
        telemetry.addData("Steps", String.format("Left: %d, Right: %d",stepsLeft,stepsRight) );
        telemetry.addData("Radius", String.format("%.0f mm from center.",radiusFromCenter_mm) );
//        telemetry.addData("rotationMode",rotationMode);
//        telemetry.addData("rotationCenter",rotationCenter); // sign of radiusFromCenter has same info. Redundant.
        telemetry.addData("arcLength", String.format("%.0f mm ",arcDistance_mm) );
        telemetry.addData("deltaHeading", String.format("%.1f deg ",deltaHeading_deg) ); // positive CCW
        telemetry.addData("RobotX", String.format("%.1f mm",deltaRobotX_mm) );
        telemetry.addData("RobotY", String.format("%.1f mm",deltaRobotY_mm) );

        // transform to game from Robot coordinates to Game Areana coordinates for: X,Y,Heading,
        //given initial robot position in game X,Y,Heading.
    }

    public void calculateResults()
    {
        double r1_mm = 0; //radius small
        double r2_mm = 0; //radius large
        int s1 = 0; //steps small
        int s2 = 0; //steps large
        double r_center_mm = 0; // positive radius to right of robot.


        // Check whether wheels are spinning the same direction or opposite directions.
        if (stepsLeft * stepsRight >= 0) {
            rotationMode = RotationMode.SAME_DIRECTION;
        }
        else {
            rotationMode = RotationMode.OPPOSITE_DIRECTION;
        }

        // Find larger/smaller step magnitude.
        if ( Math.abs(stepsRight) > Math.abs(stepsLeft) ) {
            rotationCenter = RotationCenter.LEFT_SIDE;
            s1 = Math.abs(stepsLeft);
            s2 = Math.abs(stepsRight);
        }
        else if ( Math.abs(stepsRight) < Math.abs(stepsLeft) ) {
            rotationCenter = RotationCenter.RIGHT_SIDE;
            s1 = Math.abs(stepsRight);
            s2 = Math.abs(stepsLeft);
        }
        else {
            rotationCenter = RotationCenter.CENTERED;
            s1 = Math.abs(stepsLeft);
            s2 = Math.abs(stepsRight);
        }

        // During motion through arc, will the body rotate CCW, CW, or Not at all?
        // DEBUG: Is this used at all?

        // OR, determine whether arc motion is 'forward' or 'backwards?'
        //if ( Math.max( )

        if (rotationMode == RotationMode.SAME_DIRECTION) {
            r1_mm = robot.WHEELBASE_WIDTH_MM * s1/(s2-s1);
            r2_mm = robot.WHEELBASE_WIDTH_MM + r1_mm;
            r_center_mm = (r1_mm + r2_mm)/2;
        }
        else if (rotationMode == RotationMode.OPPOSITE_DIRECTION) {
            r1_mm = robot.WHEELBASE_WIDTH_MM * s1/(s2+s1);
            r2_mm = robot.WHEELBASE_WIDTH_MM - r1_mm;
            r_center_mm = (r2_mm-r1_mm)/2;
        }
        else {
            telemetry.addData("ERROR","Wrong rotationMode value.");
        }

        // output to radiusFromCenter_mm object member
        switch (rotationCenter) {
            case LEFT_SIDE:
                radiusFromCenter_mm = -r_center_mm;
                break;
            case CENTERED:
                radiusFromCenter_mm = r_center_mm;
                break;
            case RIGHT_SIDE:
                radiusFromCenter_mm = r_center_mm;
                break;
            default:
                telemetry.addData("ERROR","DEFAULT return switch. Shouldn't happen.");
                radiusFromCenter_mm = r_center_mm;
                break;
        } // switch

        // calculate arcDistance.  Positive for robot moving 'forward'.
        // arc length of robot centerline motion.
        // calculate delta heading, X, and Y in initial robot frame.
        // RobotX to right, RobotY in driving direction. (for this class, anyhow.)
        arcDistance_mm = (stepsLeft + stepsRight)/2.0*(robot.DRIVE_WHEEL_MM_PER_ROT/robot.DRIVE_WHEEL_STEPS_PER_ROT);
        switch (rotationCenter) {
            case LEFT_SIDE:
                // same for both sides
                deltaHeading_deg = -(arcDistance_mm / radiusFromCenter_mm) * (180 /  Math.PI);
                deltaRobotY_mm =  -radiusFromCenter_mm * Math.sin(deltaHeading_deg * (Math.PI/180));
                // unique
                deltaRobotX_mm = radiusFromCenter_mm * (1 - Math.cos(deltaHeading_deg * (Math.PI/180)));
                break;
            case RIGHT_SIDE:
                // same for both sides
                deltaHeading_deg = -(arcDistance_mm / radiusFromCenter_mm) * (180 /  Math.PI);
                deltaRobotY_mm =  -radiusFromCenter_mm * Math.sin(deltaHeading_deg * (Math.PI/180));
                // unique
                deltaRobotX_mm = radiusFromCenter_mm * (1 - Math.cos(deltaHeading_deg * (Math.PI/180)));
                break;
            case CENTERED:
                switch (rotationMode) {
                    case SAME_DIRECTION:
                        deltaHeading_deg = 0;
                        deltaRobotX_mm = 0;
                        deltaRobotY_mm = arcDistance_mm;
                        break;
                    case OPPOSITE_DIRECTION:
                        deltaHeading_deg = (( stepsRight * (robot.DRIVE_WHEEL_MM_PER_ROT / robot.DRIVE_WHEEL_STEPS_PER_ROT )/r1_mm) * (180/Math.PI));
                        deltaRobotX_mm = 0;
                        deltaRobotY_mm = 0;
                        break;
                } // switch rotationMode
                break;
        } // switch rotationCenter


    } // getRadius()



} // EncoderNavigation
