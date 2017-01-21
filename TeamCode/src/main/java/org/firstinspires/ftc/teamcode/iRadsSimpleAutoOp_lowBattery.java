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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This autonomous opmode will drive forward a specified distance, shoot 2 particles, and drive the
 * rest of the way to knock off the cap ball.  Note: the launch trigger will pump 3x.  This is so
 * that if one of the balls isn't launched properly, we can recover.
 * */

@Autonomous(name="iRads Simple AutoOp (low battery)", group="iRads")  // @Autonomous(...) is the other common choice
//@Disabled
public class iRadsSimpleAutoOp_lowBattery extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    Hardware_iRads robot = new Hardware_iRads();

    private Utility utilLeftLaunchSpeed = new Utility(runtime, robot.leftLaunchMotor, 0.01); // Initialized in "initialize()" method.

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        initialize();
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);

        setLaunchPower(1);


        sleep(3000);

        //stop for a little while longer so that the launch motors can finish getting up to speed
        sleep(500);

        //launch the two particles and pull power from the launch motors and close the flippers
        launchBalls();
        setLaunchPower(0);
        robot.leftFlipper.setPosition(robot.LEFT_FLIPPER_CLOSED);
        robot.rightFlipper.setPosition(robot.RIGHT_FLIPPER_CLOSED);

        // drive forward again to knock off the cap ball and park on its platform
        driveForward(1500);

        //tells the robot to keep doing what it is doing until the drive motors are finished
        while (robot.leftDriveMotor.isBusy()){
            sleep(50);
        }
    }

    public void initialize() {
        robot.init(hardwareMap);
        resetEncoders();
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
        robot.rightDriveMotor.setPower(1);
        robot.leftDriveMotor.setPower(1);

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
        robot.leftLaunchMotor.setMaxSpeed(1050);
        robot.rightLaunchMotor.setMaxSpeed(1050);

        robot.leftLaunchMotor.setMaxSpeed(1100);
        robot.rightLaunchMotor.setMaxSpeed(1100);

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
