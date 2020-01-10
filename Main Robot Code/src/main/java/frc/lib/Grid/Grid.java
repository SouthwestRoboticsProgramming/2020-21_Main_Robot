package frc.lib.Grid;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.PID;
import frc.lib.TimeOutTimer;
import frc.lib.Looper.Loop;
import frc.lib.Looper.Looper;
import frc.robot.subsystems.DriveTrain;

public class Grid {
  private double acceleration = 1;
  private Position position = new Position();
  private Looper looper;
  private Long refreshTime = Long.valueOf("20");

  private double encodorGyroRatio = .5;
  private double angleOffset = 0;
  private Plane gyroPlane = Plane.x;
  private double maxSpeed = 1;
  private double weelBase = 0;
  private long timeOut = Long.MAX_VALUE;

  private double lastLeft = 0;
  private double lastRight = 0;

  private ADIS16448_IMU gyro = new ADIS16448_IMU();

  private DriveTrain driveTrain = new DriveTrain();
  

  /**
   * Sets a single axis position of the robot.
   *
   * @param axis the axis to be set. 
   *  @param value the selected axis position of the robot in feet from drivers station 1 corner to center of robot. Y axis is long side of field;
   */
  public void setPosition(Axis axis, double value) {
    if (axis == Axis.x) {
      position.setX(value);
    } else if (axis == Axis.y){
      position.setY(value);
    }
  }

  /**
   * Sets the position of the robot.
   *
   * @param positionX the X position of the robot in feet from drivers station 1 corner to center of robot. X axis is short side of field;
   *  @param positionY the Y position of the robot in feet from drivers station 1 corner to center of robot. Y axis is long side of field;
   */
  public void setPosition(double positionX, double positionY) {
    position.setX(positionX);
    position.setY(positionY);
  }

  /**
   * Changes a single axis position of the robot.
   *
   * @param axis the axis to be set. 
   *  @param value changes the selected axis position of the robot in feet from drivers station 1 corner to center of robot. Y axis is long side of field;
   */
  public void changePosition(Axis axis, double value) {
    if (axis == Axis.x) {
      position.changeX(value);
    } else if (axis == Axis.y){
      position.changeY(value);
    }
  }

  /**
   * Changes the position of the robot.
   *
   * @param positionX Changes the X position of the robot in feet from drivers station 1 corner to center of robot. X axis is short side of field;
   *  @param positionY Changes the Y position of the robot in feet from drivers station 1 corner to center of robot. Y axis is long side of field;
   */
  public void changePosition(double positionX, double positionY) {
    position.changeX(positionX);
    position.changeY(positionY);
  }

  /**
   * Returns the position of the robot. 
   *
   * @param axis the axis that will be returned;
   * @return the position of the robot. 
   */
  public double getPosition(Axis axis) {
    if (axis == Axis.x) {
      return position.getX();
    } else if (axis == Axis.y) {
      return position.getY();
    } else if (axis == Axis.z) {
      return position.getZ();
    } else {
      return 0;
    }
    
  }

  public enum Axis{
    x,y,z
  }

  /**
   * Sets the weel base of the robot
   *
   * @param distence the distence between the centers of the weels on a robot in feet.
   */
  public void setWeelBase(double distence) {
    weelBase = distence;
  }

  /**
   * Sets the amount of time robots moves to position before timing out. 
   *
   * @param seconds number of seconds before timeout is activated.
   */
  public void setTimeOut(Long seconds) {
    timeOut = seconds * 1000;//convert to milliseconds
  }

  /**
   * Sets the update time for tracking position. 
   *
   * @param milliseconds time in milliseconds.
   */
  public void setUpdateTime(Long milliseconds) {
    refreshTime = milliseconds;
  }

   /**
   * Sets the max robot acceleration.
   *
   * @param seconds time in second to accelerate from 0% to 100% (Assuming 20 millisecond update rate)
   */
  public void setAcceleration(double seconds) {
    this.acceleration = seconds;
  }

  /**
   * Sets the maximum output to the motors.
   *
   * @param speed between 0 & 1;
   */
  public void setMaxSpeed(double speed) {
    maxSpeed = speed;
  }

  public enum Wheel{
    Left,
    Right,
    Both
  }

  /**
   * Sets which plane of the gyro is used for the robots angle.
   *
   * @param plane whe selected plane;
   */
  public void setXPlaneGyroPlane(Plane plane) {
    gyroPlane = plane;
  }

  public enum Plane{
    x,y,z
  }

  /**
   * Sets the current physical angle of the robot.
   *
   * @param angle current angle in degrees. between -180 & 180. 
   */
  public void setCurentAngle(double angle) {
    angleOffset = angle - getAngle();
  }

  /**
   * Moves the robot based on time, not position. 
   *
   * @param leftPercent percent power being sent to left drive. between -1 & 1.
   * @param rightPercent percent power being sent to right drive. between -1 & 1.
   * @param seconds amount of time in seconds the power will be applied to weels. Acceneration, decelleration is not factored into the time, but is present. 
   */
  public void moveByTime(double leftPercent, double rightPercent, double seconds) {
      setMotorOutput(leftPercent, rightPercent);
      Timer.delay(seconds);
      setMotorOutput(0, 0);
  }
  // v1

  /**
   * Starts tracking of the position of robot on field.
   */
  private boolean startPositionTracking() {
    if (looper.getLooping()) {
      return false;
    }
    Loop loop = new Loop(){
      @Override public void onStop() {}
      @Override public void onLoop() {trackPosition();}
      @Override public void onStart() {}
      
    };
    looper = new Looper(loop, refreshTime);
    looper.start();
    return true;
  }

  /**
   * Stops tracking of the position of robot on field.
   */
  public boolean stopPositionTracking() {
    if (looper.getLooping()) {
      looper.stop();
      return true;
    } else {
      return false;
    }
  }

  /**
   * Tracks position of robot on field.
   */
  private void trackPosition() {
    double difL = driveTrain.getLeftDriveFeet() - lastLeft;
    double difR = driveTrain.getRightDriveFeet() - lastRight;

    double R;
    double r;
    if (difL < difR) {
        // reverse = true;
        R = difR;
        r = difL;
        // System.out.println("spin left");
    } else {
        // reverse = false;
        R = difL;
        r = difR;
        // System.out.println("spin right");
    }

    // angle
    if (difL != difR) {
        double x = ((r * weelBase) + Math.pow(r, 2) - (R * r)) / (R-r);
        System.out.println("x = " + x);
        double h = weelBase + r + x;
        double angleSin = Math.toDegrees(Math.asin(R/h));
        double a = 90 - angleSin - getAngle();
        double A = 90 - a;
        double b = (R + r) / 2;
        double dd= Math.sin(Math.toRadians(A));
        double d = b * dd;
        double DD = Math.sin(Math.toRadians(a));
        double D = b * DD;
        position.changeX(d);
        position.changeY(D);
    } else {
        double A = Math.sin(Math.toRadians(getAngle())) * difL;
        double B = Math.sin(Math.toRadians(90-getAngle())) * difL;
        position.changeX(A);
        position.changeY(B);
    }
    lastLeft = driveTrain.getLeftDriveEncoder();
    lastRight = driveTrain.getRightDriveEncoder();
  }

  /**
   * Moves the robot to (x,y) position on field.
   *
   * @param x position. 
   * @param y position.
   * @param dPID The pid used to move the robot closer to the specified point. double[] {kp, ki, kd};
   * @param tPID The pid used to turn the robot to face the specified point. double[] {kp, ki, kd};
   * @param tolerance how close the robot has to be to the specified point in IN for the method to complete. 
   */
    public void PIDToPosition(double x, double y, double[] dPID, double[] tPID, double tolerance) {
      TimeOutTimer timeOutTimer = new TimeOutTimer(timeOut);
          timeOutTimer.start();
      PID distence = new PID(dPID[0], dPID[1], dPID[2], "distence");
          distence.setMaxAceleration(acceleration);
          distence.setOutputLimits(maxSpeed, -maxSpeed);
          distence.setSetPoint(0);
          distence.setInverted(true);
          distence.setDisabled(false);
          distence.setError(Double.POSITIVE_INFINITY);
          distence.setSetPoint(0);
          distence.setError(pythagThem(position.getX(), position.getY(), x, y));
          distence.setActual(pythagThem(position.getX(), position.getY(), x, y));
      
      PID turn = new PID(tPID[0], tPID[1], tPID[2], "turn");
          turn.setMaxAceleration(acceleration);
          turn.setOutputLimits(maxSpeed, -maxSpeed);
          turn.setInverted(false);
          turn.setSetPoint(getTurnError(position.getX(), x, distence.getError()));

      while (Math.abs(distence.getError()) > (tolerance * 12) || !timeOutTimer.getTimedOut()) {
          turn.setSetPoint(getTurnError(position.getX(), x, distence.getError()));
          turn.setActual(getAngle());
          double turnOutput = turn.getOutput();
          double distenceOutput = distence.getOutput();
          setMotorOutput(turnOutput + distenceOutput, -turnOutput + distenceOutput);
          // System.out.println("setAngle = " + round(turn.getError(),2) + " angle = " + round(getAngle(), 2) + "negAngle = " + round(negSetAngle,2) + " posAngle = " + round(posSetAngle,2) + " output = " + round(leftOutput,2) + ", " + round(rightOutput,2));
          // System.out.println("distence = " + round(distence.getError(), 2) + " x = " + round(positionX,2) + " y = " + round(positionY,2) + " SRXL = " + round(SRXLeft , 2) + " SRXR = " + round(SRXRight, 2));
      }
      setMotorOutput(0, 0);
  }

  /**
   * turns robot to specified angle. 
   *
   * @param angle between -180 & 180.
   * @param tPID The pid used to turn the robot to face the specified angle. double[] {kp, ki, kd};
   * @param tolerance How close the robot has to be to the specified point in IN for the method to complete. 
   */
  public void turnToAngle(double angle, double[] tPID, double tolerance) {
    turnToAngle(angle, tPID, tolerance, Wheel.Both);
  }

  // v1
  /**
   * turns robot to specified angle. 
   * 
   * @param angle between -180 & 180.
   * @param tPID The pid used to turn the robot to face the specified angle. double[] {kp, ki, kd};
   * @param tolerance How close the robot has to be to the specified point in IN for the method to complete. 
   * @param wheel Which wheel turns. The opposite wheel will be the point of pivot. 
   */
  public void turnToAngle(double angle, double[] tPID, double tolerance, Wheel wheel) {
    TimeOutTimer timeOutTimer = new TimeOutTimer(timeOut);
    timeOutTimer.start();
    PID turn = new PID(tPID[0], tPID[1], tPID[2], "turn");
        turn.setMaxAceleration(acceleration);
        turn.setOutputLimits(maxSpeed, -maxSpeed);
        turn.setInverted(false);
        turn.setSetPoint(angle);
        turn.setActual(getAngle());
        turn.calcError();
    while (Math.abs(turn.getError()) > (tolerance * 12) || !timeOutTimer.getTimedOut()) {
        turn.setActual(getAngle());
        double turnOutput = turn.getOutput();
        // System.out.println("error = " + round(turn.getError(),2) + " angle = " + round(getAngle(), 2) + "negAngle = " + round(negSetAngle,2) + " posAngle = " + round(posSetAngle,2) + " output = " + round(leftOutput,2) + ", " + round(rightOutput,2));
        if (wheel == Wheel.Left) {
          setMotorOutput(turnOutput, 0);
        } else if (wheel == Wheel.Right) {
          setMotorOutput(0, -turnOutput);
        } else {
          setMotorOutput(turnOutput, -turnOutput);
        }
        
    }
    setMotorOutput(0, 0);
}

  /**
   * drives the robot straight for specified distence.
   *
   * @param feet distence in Feet that the robot will move. 
   * @param dPID The pid used to move the robot closer to the specified point. double[] {kp, ki, kd};
   * @param tPID The pid used to turn the robot to face the specified point. double[] {kp, ki, kd};
   * @param tolerance How close the robot has to be to the specified point in IN for the method to complete. 
   */
  public void driveDistence(double feet, double[] dPID, double [] tPID, double tolerance) {
    driveDistence(feet, dPID, tPID, tolerance, getAngle());
  }

  /**
   * drives the robot straight for specified distence.
   *
   * @param feet distence in Feet that the robot will move. 
   * @param dPID The pid used to move the robot closer to the specified point. double[] {kp, ki, kd};
   * @param tPID The pid used to turn the robot to face the specified point. double[] {kp, ki, kd};
   * @param tolerance How close the robot has to be to the specified point in IN for the method to complete. 
   * @param angle Angle to drive the distence. 
   */
  public void driveDistence(double feet, double[] dPID, double [] tPID, double tolerance, double angle) {
    double x = position.getX() + (Math.sin(Math.toRadians(angle)) * Math.sqrt(feet));
    double y = position.getX() + (Math.cos(Math.toRadians(angle)) * Math.sqrt(feet));
    PIDToPosition(x, y, dPID, tPID, tolerance);
  }

  // NO SMOOTHING
  private void setMotorOutput(double left, double right) {
    driveTrain.driveMotors(left, right);
  }

  // // SMOOTHING
  // private void setMotorOutputSmoothed(double left, double right) {
  //   // left
  //   double leftSmoothing = acceleration / 50;
  //   // right
  //   double rightSmoothing = acceleration / 50;

  //   double rightOutput = 0;
  //   double leftOutput = 0;
  //   while (Math.abs(leftOutput - left) > .02 && Math.abs(rightOutput - right) > .02) {
  //     Robot.driveSubsystem.driveMotors(leftOutput, rightOutput);
  //     rightOutput = rightOutput + rightSmoothing;
  //     leftOutput = leftOutput + leftSmoothing;
  //     Timer.delay(.02);
  //   }    
  //   Robot.driveSubsystem.driveMotors(left, right);
  // }

  // v1
  private double getTurnError(double y1, double y2, double distenceError) {
    double a = (y2 - y1) / distenceError;
    double c = Math.asin(a);
    double d = Math.toDegrees(c);
    double e = getAngle();
    double output = d - e;
    return output;
}

// v1
private double pythagThem(double x1, double y1, double x2, double y2) {
  double output = Math.sqrt(Math.pow((y2-y1), 2) + Math.pow((x2-x1), 2));
  return output;
}

private double getAngle() {
  double angle = 0;
  if (gyroPlane == Plane.x) {
    angle = gyro.getAngleX();
  } else if (gyroPlane == Plane.y){
    angle = gyro.getAngleY();
  } else if (gyroPlane == Plane.z) {
    angle =  gyro.getAngleZ();
  } else {
    return 0;
  }
  return angle + angleOffset;
}

// private double getSetAngle() {
//   double output = 0;
//   double ceil = (Math.ceil(output / 180));
//   if (negSetAngle != -180) {
//       output = posSetAngle - negSetAngle - 180;
//   } else {
//       output = posSetAngle;
//       ceil = 0;
//   }
//   return output - (ceil * 180);
// }
}