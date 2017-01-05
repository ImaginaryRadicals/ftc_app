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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware_iRads;

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

@Autonomous(name="iRads Simple AutoOp", group="iRads")  // @Autonomous(...) is the other common choice
//@Disabled
public class iRadsSimpleAutoOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    Hardware_iRads robot = new Hardware_iRads();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        initialize();

        // drive forward while warming up the launcher
        driveForward(1000);
        setLaunchPower(1);
        //tells the robot to keep doing what it is doing until the drive motors are finished
        while (robot.leftDriveMotor.isBusy()){
            sleep(500);
        }

        /*launch 2 balls (or "particles").  For safety, we will lift the launch trigger 3 times in
        case a ball gets trapped*/
        for(int i = 0; i < 3; i++) {
            robot.launchTrigger.setPosition(robot.ELEVATED_LAUNCHER_TRIGGER_POS);
            robot.launchTrigger.setPosition(robot.INITIAL_LAUNCHER_TRIGGER_POS);
        }
        setLaunchPower(0);
        //tells the robot to keep doing what it is doing until the launch trigger reaches its target position
        while (robot.launchTrigger.getPosition() != robot.ELEVATED_LAUNCHER_TRIGGER_POS){
            sleep(500);
        }

        // drive forward again to knock off the cap ball and park on its platform
        driveForward(1000);
        //tells the robot to keep doing what it is doing until the drive motors are finished
        while (robot.leftDriveMotor.isBusy()){
            sleep(500);
        }
    }

    public void initialize() {
        robot.init(hardwareMap);

        //other settings here
    }

    int mmToTicks(double mm) {
        double driveDistance_mm = 1000;
        double driveDistance_ticks = driveDistance_mm  * robot.DRIVE_WHEEL_STEPS_PER_ROT / robot.DRIVE_WHEEL_MM_PER_ROT;
        return (int) driveDistance_ticks;
    }

    void driveForward(double distance_mm){

        resetEncoders();
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
}
