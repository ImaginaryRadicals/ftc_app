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

    // Derivative
    // x is the independant variable, y is dependent. Returns derivative for unequally spaced points. xest is the x value at which to evaluate the derivative.
    // x and y vectors don't need to be the same length if they have three elements or more each
    static public double derivative(Vector<Double> x, Vector<Double> y, double xest) {

        int nx = x.size();
        int ny = y.size();
        if (x.size() < 2 || y.size() < 2) {
            return 0.0;
        } else if (nx == 2 || ny == 2) {
            return (y.elementAt(1) - y.elementAt(0)) / (x.elementAt(1) - x.elementAt(0));
        } else {
            double x2 = x.elementAt(nx - 1);
            double x1 = x.elementAt(nx - 2);
            double x0 = x.elementAt(nx - 3);

            double y2 = y.elementAt(ny - 1);
            double y1 = y.elementAt(ny - 2);
            double y0 = y.elementAt(ny - 3);

            double dydx = y0 * (2.0 * xest - x1 - x2) / ((x0 - x1) * (x0 - x2)) + y1 * (2.0 * xest - x0 - x2) / ((x1 - x0) * (x1 - x2)) + y2 * (2.0 * xest - x0 - x1) / ((x2 - x0) * (x2 - x1));

            return dydx;
        }
    }

    // Default derivative computation at the latest x value
    static public double derivative(Vector<Double> x, Vector<Double> y) {

        return derivative(x, y, x.lastElement());
    }
}
