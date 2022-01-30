package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Method;

public class PID {

    double _p;
    double _i;
    double _d;
    double _previous_time;
    double _previous_error;

    public PID(double p, double i, double d) {

        _p = p;
        _i = i;
        _d = d;


    }


    public double control(double target, double actual){
        double error = target - actual;
        double time = System.currentTimeMillis();
        double d_error = (error - _previous_error) / (time - _previous_time);


        return 0.0;
    }

}