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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class Trace_Route_TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Hardware_iRads robot = new Hardware_iRads();
    private ManualControl manualControl = new ManualControl();
    private InteractiveInit interactiveInit;
    private Signal sigDpadUp = new Signal();
    private Signal sigDpadB = new Signal();
    private Signal sigDpadDown = new Signal();

    private int launchSpeed = 1150;
    private boolean flippers_closed_state = false;


    @Override
    public void runOpMode() {

        initialize();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


        }
    }

    private void initialize() {
        robot.init(hardwareMap);
        interactiveInit = new InteractiveInit(telemetry, gamepad1, this);
    }

    private void updateMotors() {

        //drive backwards while holding the left trigger
        if (gamepad1.right_trigger > 0.5)
            manualControl.setSingleStickXY(gamepad1.left_stick_x, gamepad1.left_stick_y);
        else
            manualControl.setSingleStickXY((gamepad1.left_stick_x * -1.0), (gamepad1.left_stick_y * -1.0));

        //drive code
        robot.leftDriveMotor.setPower(manualControl.leftDriveMotorPower);
        robot.rightDriveMotor.setPower(manualControl.rightDriveMotorPower);

        //forklift code
        if (gamepad1.y) {
            robot.liftMotor.setPower(1.0);  // Lift up, full speed
        } else if (gamepad1.a) {
            robot.liftMotor.setPower(-1.0); // Lower down, full speed
        } else {
            robot.liftMotor.setPower(0.0);  // Lift stop.
        }

        // Adjust launchPower with d-pad up/down
        if (sigDpadUp.risingEdge(gamepad1.dpad_up)) {
            launchSpeed += 25;
        } else if (sigDpadDown.risingEdge(gamepad1.dpad_down)) {
            launchSpeed -= 25;
        }
        // Bound launchSpeed between 1300 and 900 ticks per second
        if (launchSpeed > 1300)
            launchSpeed = 1300;
        else if (launchSpeed < 900)
            launchSpeed = 900;

        // hold left_bumper to activate launcher.
        if (gamepad1.left_bumper) {
            robot.leftLaunchMotor.setPower(1);
            robot.rightLaunchMotor.setPower(1);
        } else {
            robot.leftLaunchMotor.setPower(0);
            robot.rightLaunchMotor.setPower(0);
        }

        // hold right_bumper to activate launch Trigger.
        if (gamepad1.right_bumper) {
            robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_OPEN);
            robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_OPEN);

            robot.launchTrigger.setPosition(robot.ELEVATED_LAUNCHER_TRIGGER_POS);

            //wait for the launch trigger to reach the top
            sleep(30);
            robot.leftFlipper.setPosition(robot. LEFT_FLIPPER_CLOSED);
            robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);
        }
        else {
            robot.launchTrigger.setPosition(robot.INITIAL_LAUNCHER_TRIGGER_POS);
        }

        // use b button to toggle between open and closed, hold x button to open

        if (sigDpadB.risingEdge(gamepad1.b))
        {
            flippers_closed_state = !flippers_closed_state;
        }

        if (gamepad1.x) {
            robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);
            robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        }
        else {
            if (flippers_closed_state)
            {
                robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);
                robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
            }
            else
            {
                robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_OPEN);
                robot.leftFlipper.setPosition(robot.RIGHT_FLIPPER_OPEN);
            }
        }
    }
}
