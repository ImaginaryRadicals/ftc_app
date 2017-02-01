package org.firstinspires.ftc.teamcode;

/**
 * Created by Josh Winebrener on 11/20/2016.
 */

import java.lang.Math;

public class Kinematics {

    public static final double MM_PER_INCH          = 25.4;
    //The x and the y coordinates of a target are identical
    public static final double BLUE_TARGET_XY_MM    = 2.74f * MM_PER_INCH;
    public static final double RED_TARGET_XY_MM     = -(2.74F * MM_PER_INCH);

    //
    public static double requiredLauncherVelocity(float xMm, float yMm, boolean blueTarget, double projectileHeight) {
        double launcherSpeed        = 0;
        double distanceToTarget     = 0;

        //translate origin to center of target
        if (blueTarget) {
            xMm += BLUE_TARGET_XY_MM;
            yMm += BLUE_TARGET_XY_MM;
        } else {
            xMm += RED_TARGET_XY_MM;
            yMm += RED_TARGET_XY_MM;
        }

        //use pythagorean theorem to calculate distanceToTarget
        distanceToTarget = java.lang.Math.sqrt(
                java.lang.Math.pow(xMm, 2) +
                        java.lang.Math.pow(yMm, 2)
        );


        double tanOfLauncherAngle = java.lang.Math.tan(Hardware_iRads.LAUNCHER_ANGLE_RADIANS);

        // use the pythagorean theorem to calculate the necessary diagonal velocity based on the
        // vertical and horizontal velocities
        launcherSpeed = java.lang.Math.sqrt(
                java.lang.Math.pow(
                        requiredVerticalVelocity(distanceToTarget, projectileHeight), 2
                )
                + java.lang.Math.pow(
                        requiredHorizontalVelocity(distanceToTarget, projectileHeight), 2
                )
        );

        return launcherSpeed;
    }


/* I calculated the formula for finding the necessary upward and horizontal velocities
 * based off the distanceToTarget value using the following equations:

 * tan(A_s) = V_v / V_h             //where A_s is the angle of the launcher
 * D = V_h * t                      //where V_h is horizontal velocity
 * S = -16.1t^2 + V_v * t + S_o     //where V_v is vertical velocity, S is height, and S_o
 *                                  //  is starting height
 *
 * Using these equations, we can derive...
 *
 * A_s = tan-1(V_v / V_h)
 * V_h = D / t
 * V_v = (S-S_o) / t + 16.1t
 *
 * To eliminate t, we can substitute the two second equations into the first one, then solve
 * for "t" and use that value in its place.  In order to save space and your attention span,
 * I won't type out all the steps to get here.
 *
 * t = sqrt(
 *    (tan(A_s) * D - S - S_o) / (16.1)
 * )
 *
 * If you substitute that in for "t" in the other equations, you get:
 *
 * V_h = D / (
 *    sqrt(
 *        (tan(A_s) * D - S - S_o) / 16.1
 *    )
 * )
 *
 * V_v = (S - S_o) / (
 *    sqrt(
 *        (tan(A_s) * D - S - S_o) / (16.1)
 *    )
 * ) + 16.1 * sqrt(
 *    (tan(A_s) * D - S - S_o) / (16.1)
 * )
 */

    public static double requiredTime(double distanceToTarget, double projectileHeight) {
        double requiredHorizontalVelocity   = 0;
        double launcherAngle                = Hardware_iRads.LAUNCHER_ANGLE_RADIANS;
        double launcherHeight               = Hardware_iRads.LAUNCHER_HEIGHT;
        double time                         = 0;
        //this does not represent actual time.  It will cancel out, but it makes the math look cleaner.


        time = java.lang.Math.sqrt(
                (java.lang.Math.tan(Hardware_iRads.LAUNCHER_ANGLE_RADIANS) * distanceToTarget - projectileHeight
                        - Hardware_iRads.LAUNCHER_HEIGHT) / 16.1
        );

        return time;
    }

    public static double requiredHorizontalVelocity(double distanceToTarget, double projectileHeight) {
        double requiredHorizontalVelocity = 0;

        requiredHorizontalVelocity = distanceToTarget / requiredTime(distanceToTarget, projectileHeight);

        return requiredHorizontalVelocity;
    }

    public static double requiredVerticalVelocity(double distanceToTarget, double projectileHeight) {
        double requiredVerticalVelocity = 0;

        requiredVerticalVelocity = (projectileHeight - Hardware_iRads.LAUNCHER_HEIGHT) /
                requiredTime(distanceToTarget, projectileHeight) + 16.1 * requiredTime(distanceToTarget, projectileHeight);

        return requiredVerticalVelocity;
    }
}
