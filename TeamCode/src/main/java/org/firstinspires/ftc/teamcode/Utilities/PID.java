package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.*;

import java.util.Vector;

/**
 * Created by Stephen on 2/18/17.
 * Proportional, Integral, Derivative Control
 */

public class PID
{
    private Vector<Double> time = new Vector<>(); // time history
    private Vector<Double> error = new Vector<>(); // error history
    public Double Kp = new Double(0.0); // Proportional gain
    public Double Ki = new Double(0.0); // Integral gain
    public Double Kd = new Double(0.0); // Derivative gain
    public Double integral_time = new Double(1.0);

    // pass in the current time point with the current error value
    public void addError(Double t, Double e)
    {
        time.add(t);
        error.add(e);

        while ((time.size() > 3) && (time.lastElement() - time.firstElement() > integral_time))
        {
            time.remove(0);
            error.remove(0);
        }
    }

    // retrieve the controller gain (call this after addError)
    public Double gain()
    {
        return Kp * error.lastElement() + Ki * VectorMath.integral(time, error)
                + Kd * VectorMath.derivative(time, error);
    }
}
