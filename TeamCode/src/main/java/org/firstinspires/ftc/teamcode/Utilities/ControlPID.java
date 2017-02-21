package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Stephen on 2/18/17.
 */

public class ControlPID
{
    private PID pid = new PID();
    private Mutable<Double> target = new Mutable<>(0.0);
    private Mutable<Double> value = new Mutable<>(0.0);
    private Mutable<Double> control_param = new Mutable<>(0.0);

    public ControlPID(Mutable<Double> value, Mutable<Double> target, Mutable<Double> control_param)
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
        pid.addError(t, target.get() - value.get());
        control_param.set(pid.gain());
    }
}
