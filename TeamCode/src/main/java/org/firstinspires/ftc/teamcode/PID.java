package org.firstinspires.ftc.teamcode;

import java.util.Vector;

/**
 * Created by Stephen on 2/18/17.
 * Proportional, Integral, Derivative Control
 */

public class PID
{
    private Vector<Double> time = new Vector<>();
    private Vector<Double> error = new Vector<>();
    public Double Kp = new Double(0.0);
    public Double Ki = new Double(0.0);
    public Double Kd = new Double(0.0);

    public void addError(Double t, Double e)
    {
        time.add(t);
        error.add(e);
    }

    public Double gain()
    {
        return Kp * error.lastElement() + Ki * VectorMath.integral(time, error)
                + Kd * VectorMath.derivative(time, error);
    }
}
