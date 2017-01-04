package org.firstinspires.ftc.teamcode;

/**
 * Created by Ashley on 1/2/2017.
 */

public class ManualControl {

    // Class outputs
    static double leftDriveMotorPower = 0;
    static double rightDriveMotorPower = 0;

    // singleStick control
    static void setSingleStickXY(double xStick, double yStick) {
        double magnitude = Math.sqrt( Math.pow(xStick,2)  + Math.pow(yStick,2)  );
        double angleDeg = Math.atan2(yStick,xStick) - 90;



        // Output [-1,1] where +1 is full power forward.
        if (angleDeg > 90 && angleDeg <=180) {
            // Reverse and left
            leftDriveMotorPower = (-1) * magnitude;
            rightDriveMotorPower = (1 - 2 * (angleDeg - 90) / 90) * magnitude;

        } else if(angleDeg > 0 && angleDeg <= 90) {
            // Forward and left
            leftDriveMotorPower = (1 - 2 * (angleDeg) / 90) * magnitude;
            rightDriveMotorPower = (1) * magnitude;

        } else if(angleDeg > -90 && angleDeg <= 0) {
            // Forward and right
            leftDriveMotorPower = (1) * magnitude;
            rightDriveMotorPower = (1 + 2 * (angleDeg) / 90) * magnitude;

        } else if(angleDeg >= -180 && angleDeg <= -90) {
            // Reverse and right
            leftDriveMotorPower = (1 + 2 * (angleDeg + 90) / 90) * magnitude;
            rightDriveMotorPower = (-1) * magnitude;

        } else {
            // Error
            throw new RuntimeException();
        }



    } // setSingleStickXY()


}
