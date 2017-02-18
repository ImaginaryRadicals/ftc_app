/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Utilities.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import java.util.Vector;

/**
 * Linear Autonomous Op Mode for iRads Robot.
 * This Op Mode primarily serves as a test platform for
 * both autonomous navigation and teleOp modes.
 */

@TeleOp(name="iRads: TeleOp & Nav Testmode", group="iRads")  // @Autonomous(...) is the other common choice
//@Disabled
public class iRadsDevelopmentOpMode extends LinearOpMode {


    private VisualNavigation visualNav      = new VisualNavigation();
    private EncoderNavigation encoderNav    = new EncoderNavigation();
    private ElapsedTime runtime             = new ElapsedTime();
    private NextMotorState nextMotorState   = new NextMotorState(); // Holds motor/servo cmds before sending.
    private Hardware_iRads robot            = new Hardware_iRads();   // use the class created to define iRads hardware
    private InteractiveInit interactiveInit; // Initialized in "initialize()" method.
    private Utility utilLeftLaunchSpeed; // Initialized in "initialize()" method.
    private Signal sigDpadUp                = new Signal();
    private Signal sigDpadDown              = new Signal();
    private Signal sigDpadB                 = new Signal();
    private Signal sigDpadBack              = new Signal();
    private Signal sigLeftTrigger           = new Signal();
    private Signal sigRightTrigger           = new Signal();


    double launchPower = 1.0; // Initial power of launcher.
    double expoGain = 5.0;  // 1 = no expo
    double periodSec;

    boolean flippers_closed_state = false;
    boolean runningAutonomously = false;
    boolean slowMode = false;
    boolean backwardsDrive = false;

    @Override
    public void runOpMode()
    {
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /** Start tracking the data sets we care about. */
        visualNav.visualTargets.activate();
        telemetry.addData("Status", "visualTargets Activate");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Update Period timer
            periodSec = robot.periodSec();
            telemetry.addData("Period", periodSec);

            // Navigation Code.
            visionUpdate();
            encoderUpdate();



            // Robot behavior goes here:
            //   toggle autonomous
            if (sigDpadBack.risingEdge(gamepad1.back) && runningAutonomously) {
                runningAutonomously = false;
            } else if (sigDpadBack.risingEdge() && !runningAutonomously) {
                telemetry.addData("runningAutonomously", "true");
                runningAutonomously = true;
            }

            if (runningAutonomously) {
                telemetry.addData("nextMotorState", "Autonomous");
                calculateNextAutonomousMotorState();
            } else {
                telemetry.addData("nextMotorState", "Manual");
                calculateNextMotorState();
            }
            motorUpdate();



            // Update at end of loop to display all telemetry data.
            telemetry.update();

        } // while(opModeIsActive())
    } // runOpMode()

    // Initialization
    public void initialize()
    {


        // If we can't find the hardware, print the hardware map error message and continue
        try
        {
            robot.init(hardwareMap); // Initialize Hardware (5 motors/encoders, 1 servo)
        }
        catch (IllegalArgumentException e)
        {
            telemetry.addData("Hardware map error", e.getMessage());
            robot.hardwareEnabled = false;
        }

        /*
        visualNav.initialize(runtime, telemetry); // Initialize Visual Navigation
        encoderNav.initialize(robot,runtime,telemetry);

        nextMotorState.initialize(robot, telemetry);
        if(robot.hardwareEnabled) utilLeftLaunchSpeed = new Utility(runtime, robot.leftLaunchMotor, .01);
        // Interactive Initialization Menu implementation.
        interactiveInit = new InteractiveInit(telemetry,gamepad1,this);
        interactiveInit.menuInputLoop();  // Runtime Initialization Modification.
        if (robot.hardwareEnabled) {
            if (interactiveInit.launchControl == InteractiveInit.LaunchControl.PID) {
                robot.leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (interactiveInit.launchControl == InteractiveInit.LaunchControl.POWER) {
                robot.leftLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        } // robot.hardwareEnabled == true
        encoderNav.setPosition(interactiveInit.startX,interactiveInit.startY,interactiveInit.startHeading);
        interactiveInit.displayMenu(); // Show locked parameters.
        telemetry.addData("Status", "Initialized");
        // Debug initialization display:
        if (interactiveInit.debugMode == InteractiveInit.DebugMode.ON) {
            telemetry.addLine();
            // Show all visual target goal names and coordinates.
            for (int i = 0; i<interactiveInit.goalX.size(); i++) {
                // Visual Target, robot position goal
                telemetry.addLine(interactiveInit.beaconTargets.get(i).toString());
                telemetry.addData("goalX",interactiveInit.goalX.get(i));
                telemetry.addData("goalY",interactiveInit.goalY.get(i));
                telemetry.addData("goalHeading",interactiveInit.goalHeading.get(i));
            }
            telemetry.addLine();
            visualNav.debugDisplay(); // Show navigation transformations.
        }
        telemetry.update();

        */
    }

    public void visionUpdate()
    {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        visualNav.updateTracks(VisualNavigation.DisplayMode.SHOW_OUTPUT); // vuforiaTrackables loop
        // output time since last location update.
        telemetry.addData("trackAge", String.format("%.3f",this.visualNav.getTrackAge()));
        telemetry.addData("X-value", visualNav.getX());
        telemetry.addData("Y-value", visualNav.getY());
        telemetry.addData("Heading", visualNav.getHeading());
    }

    public void encoderUpdate() {
        // Attempt to update encoder position from visualNav:
        if ( (visualNav.isNew) && (visualNav.getTrackAge() < 0.5) ) {
            visualNav.isNew = false; // Encoder already used this solution.
            encoderNav.setPosition(visualNav.getX(),visualNav.getY(),visualNav.getHeading());
        } else {
            encoderNav.setSteps();
        }
        telemetry.addData("eX",encoderNav.getX());
        telemetry.addData("eY",encoderNav.getY());
        telemetry.addData("eHeading",encoderNav.getHeading());
        telemetry.addData("absLocAge",encoderNav.getAbsoluteLocalizationAge());
        telemetry.addData("absLocDistance",encoderNav.getAbsolutePositionDistance_mm());
        telemetry.addData("locConfidence",encoderNav.getConfidence());
//        encoderNav.printResults();
    }

    public void calculateNextMotorState() {

        //Drive at half speed using boolean "slowMode".
        if (sigLeftTrigger.risingEdge(gamepad1.left_trigger > 0.5)) {
            slowMode = !slowMode;
        }
        if (slowMode == true) {
            // Left (single)stick control, Slow Mode.
            nextMotorState.leftDriveMotor = ManualControl.leftDriveMotorPower * 0.5;
            nextMotorState.rightDriveMotor = ManualControl.rightDriveMotorPower * 0.5;
            telemetry.addData("magnitude", ManualControl.magnitude);
            telemetry.addData("AngleDeg", ManualControl.angleDeg);
        } else {
            // Left (single)stick control, default.
            nextMotorState.leftDriveMotor = ManualControl.leftDriveMotorPower;
            nextMotorState.rightDriveMotor = ManualControl.rightDriveMotorPower;
            telemetry.addData("magnitude", ManualControl.magnitude);
            telemetry.addData("AngleDeg", ManualControl.angleDeg);
        }

        //Drive backwards using boolean "backwardsDrive".
        if (sigRightTrigger.risingEdge(gamepad1.right_trigger > 0.5)) {
            backwardsDrive = !backwardsDrive;
        }
        if (backwardsDrive == true) {
            // Set x and y values to negative.
            ManualControl.setSingleStickXY(gamepad1.left_stick_y, gamepad1.left_stick_x);
        } else
            ManualControl.setSingleStickXY(-gamepad1.left_stick_x, -gamepad1.left_stick_y);


        // Use gamepad buttons to move the fork lift up (Y) and down (A)
        if (gamepad1.y) {
            nextMotorState.liftMotor = 1.0;  // Lift up, full speed
        } else if (gamepad1.a) {
            nextMotorState.liftMotor = -1.0; // Lift down, full speed
        } else {
            nextMotorState.liftMotor = 0.0;  // Lift stop.
        }

        // Adjust launchPower with d-pad up/down
        if (sigDpadUp.risingEdge(gamepad1.dpad_up)) {
            launchPower += .05;
        } else if (sigDpadDown.risingEdge(gamepad1.dpad_down)) {
            launchPower -= .05;
        }
        // Bound launchPower
        launchPower = Range.clip(launchPower, 0.0, 1.0);

        // hold left_bumper to activate launcher.
        if (gamepad1.left_bumper) {
            nextMotorState.leftLaunchMotor  = launchPower;
            nextMotorState.rightLaunchMotor = launchPower;
        }
        else {
            nextMotorState.leftLaunchMotor  = 0;
            nextMotorState.rightLaunchMotor = 0;
        }

        // hold right_bumper to activate launch Trigger.
        if (gamepad1.right_bumper) {
            nextMotorState.leftFlipper = robot.LEFT_FLIPPER_OPEN;
            nextMotorState.rightFlipper = robot.RIGHT_FLIPPER_OPEN;

            nextMotorState.launchTrigger  = robot.ELEVATED_LAUNCHER_TRIGGER_POS;

            sleep(30);
            nextMotorState.leftFlipper = robot. LEFT_FLIPPER_CLOSED;
            nextMotorState.rightFlipper = robot.RIGHT_FLIPPER_CLOSED;
        }
        else {
            nextMotorState.launchTrigger  = robot.INITIAL_LAUNCHER_TRIGGER_POS;
        }

        // use b button to toggle between open and closed, hold x button to open

        if (sigDpadB.risingEdge(gamepad1.b))
        {
            flippers_closed_state = !flippers_closed_state;
        }

        if (gamepad1.x) {
            nextMotorState.rightFlipper  = robot.RIGHT_FLIPPER_CLOSED;
            nextMotorState.leftFlipper   = robot.LEFT_FLIPPER_CLOSED;
        }
        else {
            if (flippers_closed_state)
            {
                nextMotorState.rightFlipper  = robot.RIGHT_FLIPPER_CLOSED;
                nextMotorState.leftFlipper   = robot.LEFT_FLIPPER_CLOSED;
            }
            else
            {
                nextMotorState.rightFlipper  = robot.RIGHT_FLIPPER_OPEN;
                nextMotorState.leftFlipper  = robot.RIGHT_FLIPPER_OPEN;
            }
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", nextMotorState.leftDriveMotor);
        telemetry.addData("right", "%.2f", nextMotorState.rightDriveMotor);
        telemetry.addData("Left Flipper", "%.2f", nextMotorState.leftFlipper);
        telemetry.addData("Right Flipper", "%.2f", nextMotorState.rightFlipper);
        telemetry.addData("Ideal Launch Speed", "%.2f", nextMotorState.leftLaunchMotor*Hardware_iRads.MAX_LAUNCH_SPEED_TPS);
        if (robot.hardwareEnabled) {
            telemetry.addData("Actual Launch Speed", "%.2f", utilLeftLaunchSpeed.getMotorTickRate());
        }


    } // calculateNextMotorState()

    public void calculateNextAutonomousMotorState() {
        telemetry.addData("Hey guys", "Running Autonomously!");
        double autonomousGoalHeading = 90.0; // The robot will turn to this value in the auto mode
        double autonomousGoalHeadingTolerance = 0.0;
        double maxSpeed = 0.5;
        double angleStartRampDown = 10; // Degrees from goalHeading.
        double errorSignal = Math.abs(encoderNav.getHeading() - autonomousGoalHeading) / angleStartRampDown;
        errorSignal = Range.clip(errorSignal, 0, 1);

        if(encoderNav.getHeading() < (autonomousGoalHeading - autonomousGoalHeadingTolerance)) {

            nextMotorState.leftDriveMotor = -maxSpeed * errorSignal;
            nextMotorState.rightDriveMotor = maxSpeed * errorSignal;

        } else if(encoderNav.getHeading() > autonomousGoalHeading + autonomousGoalHeadingTolerance) {

            nextMotorState.leftDriveMotor = maxSpeed * errorSignal;
            nextMotorState.rightDriveMotor = -maxSpeed * errorSignal;

        } else {
            telemetry.addData("Alert", "Heading is 0 correct.");
        }
    }

    public void motorUpdate()
    {
        if (robot.hardwareEnabled) {
            nextMotorState.updateMotors();
        } else {
            nextMotorState.print(); // Display commands if hardware is disabled.
        }

    }
} // Class
