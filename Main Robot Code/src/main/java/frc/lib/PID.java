package frc.lib;
import java.util.Calendar;
// PID v1
public class PID{
    // log
    private String name = "";

    // PID coefficients
    private double p = 0, i = 0, d = 0, f = 0, fAngle = 0, fTPR = 0;
    FType fType;

    // Dynamic variables
    private double lastUpdate = Double.NaN;
    private double lastSetPoint = Double.NaN;
    private double lastError = 0;
    private double lastOutput = 0;
    private double lastActual = Double.NaN;
    private double integralError = 0;
    private double pids = 1;

    // max / min Limits
    private double maxLimit = Double.POSITIVE_INFINITY;
    private double minLimit = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxAcceleration = Double.POSITIVE_INFINITY;
    private boolean inverted = false; 
    private boolean disabled = false; 


    /**
     * Setup PID Control
     * 
     * @param p The proportional gain coefficient.
     * @param i The integral gain coefficient.
     * @param d The derivative gain coefficient.
     */
	public PID(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }

    /**
     * Setup PID Control
     * 
     * @param p The proportional gain coefficient.
     * @param i The integral gain coefficient.
     * @param d The derivative gain coefficient.
     * @param name The name shown when an error occurs.
     */
	public PID(double p, double i, double d, String name){
        this.p = p;
        this.i = i;
        this.d = d;
        this.name = name;
    }
    
    /**
	 * @param p The proportional gain coefficient.
     * @param i The integral gain coefficient.
     * @param d The derivative gain coefficient.
	 */
	public void setPID(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }

	/**
	 * @param p The proportional gain coefficient.
	 */
	public void setP(double p){this.p = p;}

	/**
	 * @param i The integral gain coefficient.
	 */
	public void setI(double i){this.i = i;}

	/**
	 * @param d The derivative gain coefficient.
	 */
    public void setD(double d){this.d = d;}

    public enum FType {
        fixed,
        rotary
    }

    /**
	 * @param type How f will be used based on how it is applied.
     * @param value Value of f, when type is fixed.
	 */
    public void setF(FType type, double value){
        fType = FType.fixed;
        f = value;
        
    }

    /**
	 * @param type How f will be used based on how it is applied.
     * @param value Value of f when type is rotary.
     * @param angle Angle at which value was sampled. The closer f is to level (90 degrees), the more acorite f will be.
     * @param ticksPerRevolution How many tick per revolution of encoder / gyro. 
	 */
    public void setF(FType type, double value, double angle, double ticksPerRevolution){
        fType = FType.rotary;
        f = value;
        fAngle = angle;
        fTPR = ticksPerRevolution;
    }
    
    /**
	 * @param name the name shown when an error occurs.
	 */
    public void setName(String name){this.name = name;}
    
	/**
	 * Set max & min output.
	 * @param minimum possible output value
	 * @param maximum possible output value
	 */
	public void setOutputLimits(double maxOutput, double minOutput){
        if (maxOutput < minOutput) {
            printError("PID output limits out of range!");
            return;
        }   
        this.maxOutput = maxOutput;
        this.minOutput = minOutput;
    }
    
    /**
	 * Set min output.
	 * @param minimum possible output value
	 */
	public void setMinOutputLimit(double minOutput){
        setOutputLimits(maxOutput, minOutput);
    }
    
    /**
	 * Set min output.
	 * @param maximum possible output value
	 */
	public void setMaxOutputLimit(double maxOutput){
        setOutputLimits(maxOutput, minOutput);
	}

	/** 
	 * Set the operating direction of the PID controller
	 * @param reversed Set true to reverse PID output
	 */
	public void setInverted(boolean inverted){
		this.inverted = inverted;
    }
    
    /** 
	 * Sets temp error for pid control.
	 * @param error the err that will be set. 
	 */
	public void setError(double error){
		lastError = error;
    }

    /** 
	 * Set if the output is disabled.
     * when the output is disabled, the output will be 0.
	 * @param disabled Set true to disable output. 
	 */
	public void setDisabled(boolean disabled){
		this.disabled = disabled;
    }
    
    /** 
     * @return if PID is disabled.
	 */
	public boolean getDisabled(){
        return disabled;
	}

    /** 
     * calculates the error between the setpoint and actual position.
	 */
	public void calcError(){
        lastError = lastSetPoint - lastActual;
	}

	/**
	 * Calculate the output value.
	 * @param actual The current value of the sencer input. 
	 * @param setpoint The set value of the sensor input. 
	 * @return calculated output value.
	 */
	public double getOutput(double actual, double setPoint){
        // check
        setPoint = checkMinMax(setPoint, minLimit, maxLimit);
        if (checkEqual(actual, Double.NaN)) {
            printError("Actual not set!");
            return Double.NaN;
        }

        if (checkEqual(setPoint, Double.NaN)) {
            printError("setPoint not set!");
            return Double.NaN;
        }

        if (checkEqual(integralError, Double.NaN)) {
            integralError = 0;
        }

        if (disabled) {
            return 0;
        }

        // get baseTime
            double baseTime = getTime();

        // calc proportional
            double error = setPoint - actual;
        double outputP = p * error;

        if (checkEqual(outputP, Double.NaN)) {
            outputP = 0;
        }

            double tbt = baseTime - lastUpdate;
            double dt = (lastUpdate != Double.NaN) ? (double)(tbt) : 0;

        // calc integral
        integralError += error * dt; 
        double outputI = i * integralError * .0001;

        if (checkEqual(outputI, Double.NaN)) {
            outputI = 0;
        }

        // calc derivative
            double derivativeError = (dt != 0) ? ((error - lastError) / dt) : 0;
        double outputD = d * derivativeError;

        if (checkEqual(outputD, Double.NaN)) {
            outputD = 0;
        }

        double outputF = getF(actual);

        // calc sum of PID
        double total = outputP + outputI + outputD + outputF;
        total = checkMinMax(total, minOutput, maxOutput);
        total = checkMaxChange(lastOutput, total, maxAcceleration / pids);
        if (inverted) {total = -total;}

        // store for next run
        lastUpdate = baseTime;
        lastError = error;
        lastOutput = total;
        lastSetPoint = setPoint;

        return total;
    }

    /**
	 * Calculate the output value.
	 * @param actual The current value of the sencer input. 
	 * @return calculated output value.
	 */
	public double getOutput(double actual){
        if (checkEqual(lastSetPoint, Double.NaN)) {
            printError("PID setpoint not initualized!");
            return Double.NaN;
        }
		return getOutput(actual, lastSetPoint);
    }

    /**
     * Calculate the output value.
     * @return calculated output value.
     */
    public double getOutput() {
        return getOutput(lastActual, lastSetPoint);
    }

    /**
     * Returns last known error between set & actual position.
     * @return calculated output value.
     */
    public double getError() {
        return lastError;
    }
    
    /**
	 * Sets the setPoint value.
	 * @param setPoint The set value of the sensor input. 
	 */
	public void setSetPoint(double setPoint){
        this.lastSetPoint = setPoint;
    }
    
    /**
	 * Sets the current value of the senser input. 
	 * @param actual The current value of the senser input. 
	 */
	public void setActual(double actual){
        this.lastActual = actual;
	}
	
	/**
	 * Resets all values to default.
	 */
	public void reset(){
        p = 0;
        i = 0;
        d = 0;
        f = 0;
        fAngle = 0;
        fTPR = 0;
        fType = FType.fixed;
        lastUpdate = Double.NaN;
        lastSetPoint = Double.NaN;
        lastError = 0;
        lastOutput = 0;
        lastActual = Double.NaN;
        integralError = 0;
        pids = 1;
        maxLimit = Double.POSITIVE_INFINITY;
        minLimit = Double.NEGATIVE_INFINITY;
        maxOutput = Double.POSITIVE_INFINITY;
        minOutput = Double.NEGATIVE_INFINITY;
        maxAcceleration = Double.POSITIVE_INFINITY;
        inverted = false;     
    }
    
	/**
     * Sets the maximum acceleration. 
	 * @param acceleration time of output to increase by 1 unit; 
	 */
	public void setMaxAceleration(double acceleration){
		maxAcceleration = Math.abs(acceleration / 50);
    }


    
    //-------------------------------------------- 
    // Private
    //--------------------------------------------

    private double getF(double ticksAngle) {
        if (fType == FType.fixed) {
            return f;
        } else if (fType == FType.rotary) {
            double angle = ticksAngle / fTPR * 360;
            double accountedF = f / Math.sin(Math.toRadians(fAngle));
            return Math.sin(Math.toRadians(angle)) * accountedF;
        } else {
            return 0;
        }
    }

    private double checkMinMax(double var, double min, double max) {
        return Math.max(Math.min(var, max), min);
    }

    public boolean checkEqual(Object a, Object b) {
        if (String.valueOf(a).equals(String.valueOf(b))) {
            return true;
        } else {
            return false;
        }
    }

    private double checkMaxChange(double var1, double var2, double change) {
        double output;
        if (var2 - var1 > change) {
            output = var1 + change;
        } else if (var2 - var1 < -change) {
            output = var1 - change;
        } else {
            output = var2;
        }
        return output;
    }

    private void printError(String error) {
        String ANSI_RESET = "\u001B[0m";
        String ANSI_RED = "\u001B[31m";
        System.out.println(ANSI_RED + name + " " + error + ANSI_RESET);
    }

    private long getTime() {
        return Calendar.getInstance().getTimeInMillis();
    }

}