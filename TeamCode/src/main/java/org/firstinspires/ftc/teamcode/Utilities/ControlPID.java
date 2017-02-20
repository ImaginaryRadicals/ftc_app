package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Stephen on 2/18/17.
 */

public class ControlPID
{
    private PID pid = new PID();
    private Double target = new Double(0.0);
    private Double value = new Double(0.0);
    private Double control_param = new Double(0.0);

    public ControlPID(Double value, Double target, Double control_param)
    {
        this.value = value;
        this.target = target;
        this.control_param = control_param;
    }

    public void setGains(Double Kp, Double Ki, Double Kd)
    {
        pid.Kp = Kp;
        pid.Ki = Ki;
        pid.Kd = Kd;
    }

    public void setIntegralTime(Double integral_time)
    {
        pid.integral_time = integral_time;
    }

    public void update(Double t)
    {
        pid.addError(t, target - value);
        control_param = pid.gain();
    }
}
