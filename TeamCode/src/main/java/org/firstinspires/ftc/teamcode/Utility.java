package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by aferg on 11/25/2016.
 */

public class Utility {
    static double expo(double input, double gain) {
        // y = (x^3 * (k-1) + x ) / k
        // formula for exponential drive
        gain = Range.clip(gain, 1,1000);
        double output = ((Math.pow(input, 3) * (gain-1) + input)/gain);
        return Range.clip(output, -1,1);

    }
}
