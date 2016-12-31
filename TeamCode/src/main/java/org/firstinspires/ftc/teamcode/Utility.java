package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Util;

/**
 * Created by aferg on 11/25/2016.
 */

public class Utility {

    // Static properties
    public static ElapsedTime runtime;

    // Instance properties
    Vector pastMotorRates = new Vector();
    double lastEncoderPosition = 0;
    double lastTime = 0;
    DcMotor dcMotor;
    double updatePeriod = 0.01; //


    public Utility(ElapsedTime runtime, DcMotor dcMotor, double updatePeriod) {
        this.runtime = runtime;
        this.dcMotor = dcMotor;
        this.updatePeriod = updatePeriod;
    }


    /**
     * @param input is a joystick input from (-1,1)
     * @param gain is expected to be (1,1000), where 1 has no expo at all.
     * @return output in range (-1,1)
     */
    static double expo(double input, double gain) {
        // y = (x^3 * (k-1) + x ) / k
        // formula for exponential drive
        gain = Range.clip(gain, 1,1000);
        double output = ((Math.pow(input, 3) * (gain-1) + input)/gain);
        return Range.clip(output, -1,1);
    }



    public double getMotorTickRate() {
        // "Inputs"
        DcMotor motorInQuestion = this.dcMotor;
        double currentTime = runtime.time();

        double currentEncoderPosition = motorInQuestion.getCurrentPosition();
        double deltaEncoderPositions = currentEncoderPosition - lastEncoderPosition;
        double deltaElapsedTime = currentTime - lastTime;
        double instTickRate;
        double avgTickRate;
        int avgSamples = 30;

        if (deltaElapsedTime != 0) { // Avoid divide by zero.
            instTickRate = deltaEncoderPositions / deltaElapsedTime;
        } else {
            instTickRate = 0;
        }

        pastMotorRates.add(instTickRate);

        if (pastMotorRates.size() >= avgSamples) {
            pastMotorRates.remove(0);
        }

        lastEncoderPosition = currentEncoderPosition;
        lastTime = currentTime;

        avgTickRate = VectorMath.average(pastMotorRates);
        return avgTickRate;
    }
}
