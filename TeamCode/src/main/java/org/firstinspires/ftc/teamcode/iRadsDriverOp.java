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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.EncoderNavigation;
import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;
import org.firstinspires.ftc.teamcode.Utilities.ManualControl;
import org.firstinspires.ftc.teamcode.Utilities.NextMotorState;
import org.firstinspires.ftc.teamcode.Utilities.Signal;
import org.firstinspires.ftc.teamcode.Utilities.Utility;
import org.firstinspires.ftc.teamcode.Utilities.VisualNavigation;

/**
 * Competitoin TeleOp mode for iRads Robot.
 * Simplified from iRadsAutoOpMode_linear.java
 *
 */

@TeleOp(name="iRads: DriverOp", group="iRads")  // @Autonomous(...) is the other common choice
//@Disabled
public class iRadsDriverOp extends LinearOpMode {


    private ElapsedTime runtime             = new ElapsedTime();
    private NextMotorState nextMotorState   = new NextMotorState(); // Holds motor/servo cmds before sending.
    private Hardware_iRads robot            = new Hardware_iRads();   // use the class created to define iRads hardware
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
    boolean slowMode = false;
    boolean backwardsDrive = false;

    @Override
    public void runOpMode()
    {
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Update Period timer
            periodSec = robot.periodSec();
            telemetry.addData("Period", periodSec);

            // Robot behavior goes here:
            calculateNextMotorState();
            motorUpdate();

            // Update at end of loop to display all telemetry data.
            telemetry.update();

        } // while(opModeIsActive())
    } // runOpMode()

    // Initialization
    public void initialize()
    {
        telemetry.addData("Warning:", " You forgot to initialize gamepad1.");

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

        
        nextMotorState.initialize(robot, telemetry);
        if(robot.hardwareEnabled) utilLeftLaunchSpeed = new Utility(runtime, robot.leftLaunchMotor, .01);
    }


    public void calculateNextMotorState() {

        updateDriveMotors();
        updateForklift();
        updateLaunchMotors();
        updateLaunchTrigger();
        updateFlippers();
        updateTelemetry();

    } // calculateNextMotorState()


    public void motorUpdate()
    {
        if (robot.hardwareEnabled)  nextMotorState.updateMotors();
        else                        nextMotorState.print(); // Display commands if hardware is disabled.

    }

    private void updateDriveMotors() {

        updateSlowDrive();
        updateReverseDrive();

    }

    private void updateReverseDrive() {

        //Drive backwards using boolean "backwardsDrive".
        if (sigRightTrigger.risingEdge(gamepad1.right_trigger > 0.5))
            backwardsDrive = !backwardsDrive;

        if (backwardsDrive == true) {

            ManualControl.setSingleStickXY(-gamepad1.left_stick_x, -gamepad1.left_stick_y);

        } else {

            ManualControl.setSingleStickXY(gamepad1.left_stick_x, gamepad1.left_stick_y);

        }

    }

    private void updateSlowDrive() {

        //Drive at half speed using boolean "slowMode".
        if (sigLeftTrigger.risingEdge(gamepad1.left_trigger > 0.5))     slowMode = !slowMode;

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

    }

    private void updateForklift() {

        // Use gamepad buttons to move the fork lift up (Y) and down (A)
        if (gamepad1.y)         nextMotorState.liftMotor = 1.0;  // Lift up, full speed
        else if (gamepad1.a)    nextMotorState.liftMotor = -1.0; // Lift down, full speed
        else                    nextMotorState.liftMotor = 0.0;  // Lift stop.

    }

    private void updateLaunchMotors() {

        // Adjust launchPower with d-pad up/down
        if (sigDpadUp.risingEdge(gamepad1.dpad_up))             launchPower += .05;
        else if (sigDpadDown.risingEdge(gamepad1.dpad_down))    launchPower -= .05;

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

    }

    private void updateLaunchTrigger() {

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

    }

    private void updateFlippers() {

        // use b button to toggle between open and closed, hold x button to open
        if (sigDpadB.risingEdge(gamepad1.b))    flippers_closed_state = !flippers_closed_state;

        if (gamepad1.x) {

            nextMotorState.rightFlipper  = robot.RIGHT_FLIPPER_CLOSED;
            nextMotorState.leftFlipper   = robot.LEFT_FLIPPER_CLOSED;

        }

        else {

            if (flippers_closed_state) {

                nextMotorState.rightFlipper  = robot.RIGHT_FLIPPER_CLOSED;
                nextMotorState.leftFlipper   = robot.LEFT_FLIPPER_CLOSED;

            } else {

                nextMotorState.rightFlipper  = robot.RIGHT_FLIPPER_OPEN;
                nextMotorState.leftFlipper  = robot.RIGHT_FLIPPER_OPEN;
            }

        }

    }

    private void updateTelemetry() {

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", nextMotorState.leftDriveMotor);
        telemetry.addData("right", "%.2f", nextMotorState.rightDriveMotor);
        telemetry.addData("Left Flipper", "%.2f", nextMotorState.leftFlipper);
        telemetry.addData("Right Flipper", "%.2f", nextMotorState.rightFlipper);
        telemetry.addData("Ideal Launch Speed", "%.2f", nextMotorState.leftLaunchMotor*Hardware_iRads.MAX_LAUNCH_SPEED_TPS);

        try {

            if (robot.hardwareEnabled)
                telemetry.addData("Actual Launch Speed", "%.2f", utilLeftLaunchSpeed.getMotorTickRate());


        } catch (NullPointerException npe) {

            telemetry.addData("error", "could not find launch motor tick rate");
            telemetry.addLine();
            telemetry.addData("here was the error", npe.getMessage());

        }

    }

} // Class
