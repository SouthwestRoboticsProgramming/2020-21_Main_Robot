package frc.lib;

public class Lib{

    /**
     * Returns the absolute of the diffrence between the specified doubles
     * 
     * @param a First number.
     * @param b Second number.
     */
    public double diffOfNumbers(double a, double b){
        return Math.abs(a-b);
    }

    public double setRange(double var, double min, double max) {
        return Math.max(Math.min(var, max), min);
    }
}