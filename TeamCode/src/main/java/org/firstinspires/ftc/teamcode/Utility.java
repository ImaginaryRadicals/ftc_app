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

        double currentTime = runtime.time();
        double avgTickRate;

        if(currentTime > lastTime + updatePeriod) {

            DcMotor motorInQuestion = this.dcMotor;
            double currentEncoderPosition = motorInQuestion.getCurrentPosition();
            double deltaEncoderPositions = currentEncoderPosition - lastEncoderPosition;
            double deltaElapsedTime = currentTime - lastTime;
            double instTickRate;
            int avgSamples = 30;

            // Calculate instantaneous rate. Don't divide by zero.
            if (deltaElapsedTime != 0) {
                instTickRate = deltaEncoderPositions / deltaElapsedTime;
            } else {
                instTickRate = 0;
            }

            // Add new instantaneous rate to pastMotorRates vector. Keep vector length below limit.
            pastMotorRates.add(instTickRate);
            if (pastMotorRates.size() >= avgSamples) {
                pastMotorRates.remove(0);
            }

            lastEncoderPosition = currentEncoderPosition;
            lastTime = currentTime;
        } // if updatePeriod elapsed

        avgTickRate = VectorMath.average(pastMotorRates);
        return avgTickRate;

    } // getMotorTickRate()

} // Utility class
