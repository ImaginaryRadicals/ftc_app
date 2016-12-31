package org.firstinspires.ftc.teamcode;

import java.util.Vector;

/**
 * Created by Stephen on 12/10/16.
 */

public class VectorMath
{
    // Summation
    static public double sum(Vector<Double> v)
    {
        double ret = 0.0;
        for (int i = 0; i < v.size(); ++i)
        {
            ret += v.get(i);
        }
        return ret;
    }

    // Get the average value of an entire vector
    static public double average(Vector<Double> v)
    {
        if (v.size() > 0)
        {
            return sum(v) / v.size();
        }
        else
        {
            return 0.0;
        }
    }

    // Get the average over some past number of steps
    static public double average(Vector<Double> v, int steps)
    {
        int size = v.size();
        int start = size - steps;

        double ret = 0.0;
        for (int i = start; i < size; ++i)
        {
            ret += v.get(i);
        }

        ret /= size - steps;

        return ret;
    }

    // Get the span of the vector, i.e. lastValue - firstValue.
    static public double span(Vector<Double> v) {
        if (v.size() > 0)
        {
            return v.get(v.size()-1) - v.get(0);
        }
        else
        {
            return 0.0;
        }
    }
}
