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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Linear Autonomous Op Mode for iRads Robot.
 * Currently, only vision is implemented. Efforts have been made to move Vuforia code to
 * the VisualNavigation class, which is instantiated as 'visualNav' in this OpMode.
 *
 * Hardware has been instantiated using the Hardware_iRads class, but has not been initialized.
 * To initialize, simply uncomment 'robot.init(hardwareMap)' in the initialization block below.
 *
 */

@Autonomous(name="iRads: AutoOp Vision Linear", group="iRads")  // @Autonomous(...) is the other common choice
//@Disabled
public class iRadsAutoOpMode_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private VisualNavigation visualNav = new VisualNavigation();
    private EncoderNavigation encoderNav= new EncoderNavigation();
    private ElapsedTime runtime = new ElapsedTime();
    private NextMotorState nextMotorState = new NextMotorState(); // Holds motor/servo cmds before sending.
    Hardware_iRads robot = new Hardware_iRads();   // use the class created to define iRads hardware

    double launchPower = 1.0; // Initial power of launcher.

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
            // Navigation Code.
            visionUpdate();
            encoderUpdate();



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
        // If we can't find the hardware, print the hardware map error message and continue
        try
        {
            robot.init(hardwareMap); // Initialize Hardware (5 motors/encoders, 1 servo)
        }
        catch (IllegalArgumentException e)
        {
            telemetry.addData("Hardware map error", e.getMessage());
        }
        visualNav.initialize(runtime, telemetry); // Initialize Visual Navigation
        encoderNav.initialize(robot,runtime,telemetry);
        encoderNav.setPosition(1000,1000,90);
        nextMotorState.initialize(robot);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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

        // Left and Right sticks control tank drive.
        nextMotorState.leftDriveMotor  = -gamepad1.left_stick_y;
        nextMotorState.rightDriveMotor = -gamepad1.right_stick_y;

        // Use gamepad buttons to move the fork lift up (Y) and down (A)
        if (gamepad1.y) {
            nextMotorState.liftMotor = 1.0;  // Lift up, full speed
        } else if (gamepad1.a) {
            nextMotorState.liftMotor = -1.0; // Lift down, full speed
        } else {
            nextMotorState.liftMotor = 0.0;  // Lift stop.
        }

        // Adjust launchPower with d-pad up/down
        if (gamepad1.dpad_up) {
            launchPower += .05;
        } else if (gamepad1.dpad_down) {
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


        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", nextMotorState.leftDriveMotor);
        telemetry.addData("right", "%.2f", nextMotorState.rightDriveMotor);
        telemetry.addData("LaunchSpeed", "%.2f", nextMotorState.leftLaunchMotor*Hardware_iRads.MAX_LAUNCH_SPEED_TPS);

    } // calculateNextMotorState()

    public void motorUpdate()
    {
        // nextMotorState.updateMotors();
        robot.leftDriveMotor.setPower(nextMotorState.leftDriveMotor);
        robot.rightDriveMotor.setPower(nextMotorState.rightDriveMotor);
        robot.leftLaunchMotor.setPower(nextMotorState.leftLaunchMotor);
        robot.rightLaunchMotor.setPower(nextMotorState.rightLaunchMotor);
        robot.liftMotor.setPower(nextMotorState.liftMotor);
        robot.buttonPusher.setPosition(nextMotorState.buttonPusher);
    }
} // Class
