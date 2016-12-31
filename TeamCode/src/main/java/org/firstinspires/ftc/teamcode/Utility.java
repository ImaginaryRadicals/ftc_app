package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import java.util.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by aferg on 11/25/2016.
 */

public class Utility {

    Vector pastMotorRates = new Vector();
    double lastEncoderPosition = 0;
    double lastElapsedTime = 0;

    static double expo(double input, double gain) {
        // y = (x^3 * (k-1) + x ) / k
        // formula for exponential drive
        gain = Range.clip(gain, 1,1000);
        double output = ((Math.pow(input, 3) * (gain-1) + input)/gain);
        return Range.clip(output, -1,1);

    }

    public double getMotorTickRate(DcMotor motorInQuestion, double elapsedTime) {

        double currentEncoderPosition = motorInQuestion.getCurrentPosition();
        double motorTickRate = 0;
        double currentElapsedTime = 0;

        double deltaEncoderPositions = currentEncoderPosition - lastEncoderPosition;
        double deltaElapsedTime = currentElapsedTime - lastElapsedTime;
        double rate = 0;
        if (deltaElapsedTime != 0) { // Avoid divide by zero.
            rate = deltaEncoderPositions / deltaElapsedTime;
        }

        pastMotorRates.add(rate);

        if (pastMotorRates.size() >= 30) {
            pastMotorRates.remove(0);
        }

        motorTickRate = VectorMath.average(pastMotorRates);

        lastEncoderPosition = currentEncoderPosition;
        lastElapsedTime = currentElapsedTime;

        return motorTickRate;
    }
}
