/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class Waypoint {
    public double theta, m, x, y, a, b, c, d, e, f;
    public double low, high, distance;
    public Function<Double,Double> path;
    public Waypoint(double heading, double x, double y) {
        this.theta = heading;
        this.x = x;
        this.y = y;
        this.m = Math.atan(theta);
    }
    public Waypoint join(Waypoint end) {
        distance = Math.hypot(this.x - end.x, this.y-end.y);
        a = (60 * m - 39 * end.m) / (70 * Math.pow(distance,3));
        b = (end.m - m) * Math.pow(distance,-3);
        c = -(4*end.m + 6 * m) * Math.pow(d,-2) /7d;
        d = 0;
        e = m;
        f = 0;
    }
    public path(double x) {
        return quintic(a,b,c,d,e,f).apply(x);
    }
    public Function<Double,Double> quintic(double a_, double b_, double c_, double e_, double f_) {
        return x -> a_ * Math.pow(x,5) + b_ * Math.pow(x,4) + c_ * Math.pow(x,3) + d_ * Math.pow(x,2) + e_ * x + f_;
    }
    public Function<Double,Double> derivate(Function<Double,Double> fn) {
        return x -> (fn.apply(Math.nextUp(x)) - fn.apply(Math.nextDown(x))) / (Math.nextUp(x) - Math.nextDown(x))
    }
    public double slope() {
        double disX = Robot.in.encoders.x() / Math.cos(theta);
        double disY = Robot.in.encoders.y() / Math.sin(theta);
        double dis  = (disX + disY) / 2
        return 5 * a * Math.pow(dis,4) + 4 * b * Math.pow(dis,3) + 3 * c * Math.pow(dis,2) + 2 * d * dis + m;
    }
    public Function<Double,Double> omega() {
        return x -> Math.atan(derivate(quintic(a,b,c,d,e,f)).apply(x)); // maybe have to add another term;
    }

    Waypoint(90,0,0).join(new Waypoint(180,12,3)).join(2rjopijoiassdnf).
}
