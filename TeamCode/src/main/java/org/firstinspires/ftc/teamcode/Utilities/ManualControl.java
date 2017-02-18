package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.*;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ashley on 1/2/2017.
 */

public class ManualControl {

    // Class outputs
    public static double leftDriveMotorPower = 0;
    public static double rightDriveMotorPower = 0;
    public static double magnitude;
    public static double angleDeg;


    // singleStick control
    public static void setSingleStickXY(double xStick, double yStick) {
        yStick = -yStick; // flip y axis. -1 was forward.
        magnitude = Math.sqrt( Math.pow(xStick,2)  + Math.pow(yStick,2)  );
        magnitude = Range.clip(magnitude,-1,1);
        angleDeg = Math.atan2(yStick,xStick)*180/Math.PI;

        // Prevent negative overflow.
        if (angleDeg <=-90) {
            angleDeg = angleDeg + 360;
        }
        angleDeg = angleDeg - 90; // Rotate so 0 degrees is in the + y direction (forward).



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
            //throw new RuntimeException();

        }



    } // setSingleStickXY()


}
