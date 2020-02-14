package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Looper.Looper;
import frc.robot.Constants;
import frc.robot.Robot;

//TODO: Clarify lift, lower, block, and unblock in solenoid names. Difficult to understand what does what.

/*
* Ball subsystem includes all motors, solenoids, and sensors assosiated with picking up and storing balls.
  Note that "lower" refers to the part of the belt system near the intake, while "upper" refers to the other end.
*/
public class BallSubsystem extends SubsystemBase {
  private final WPI_TalonSRX intakeTalon, beltTalon;
  private final WPI_VictorSPX flickerVictor, outputVictor;
  private final DoubleSolenoid intakeDoubleSolenoid, lowerBlockDoubleSolenoid, upperBlockDoubleSolenoid;
  private final DigitalInput lowerBallSensor, upperBallSensor;

  private int storedBalls = 0;
  private final int maxStoredBalls = 5;
  private ballMode mode;
  private Looper ballLooper;

  private boolean lowerBallSensorBlocked = false,
                  upperBallSensorBlocked = false;

  public BallSubsystem() {
    intakeTalon = new WPI_TalonSRX(Constants.intakeTalonPort);
    intakeTalon.setInverted(false);
    flickerVictor = new WPI_VictorSPX(Constants.flickerVictorPort);
    flickerVictor.setInverted(false);
    beltTalon = new WPI_TalonSRX(Constants.beltVictorPort);
    beltTalon.setInverted(true);
    outputVictor = new WPI_VictorSPX(Constants.outputVictorPort);
  
    intakeTalon.setNeutralMode(NeutralMode.Brake);
    flickerVictor.setNeutralMode(NeutralMode.Brake);
    beltTalon.setNeutralMode(NeutralMode.Brake);
    outputVictor.setNeutralMode(NeutralMode.Brake);

    intakeDoubleSolenoid = new DoubleSolenoid(37, Constants.lowerIntakeSolenoidPort, Constants.liftIntakeSolenoidPort);
    lowerBlockDoubleSolenoid = new DoubleSolenoid(37, Constants.closeLowerSolenoidPort, Constants.openLowerSolenoidPort);
    upperBlockDoubleSolenoid = new DoubleSolenoid(37, Constants.closeUpperSolenoidPort, Constants.openUpperSolenoidPort);
    intakeDoubleSolenoid.set(Value.kReverse);
    lowerBlockDoubleSolenoid.set(Value.kOff);
    upperBlockDoubleSolenoid.set(Value.kOff);

    lowerBallSensor = new DigitalInput(Constants.lowerBallSensorPort);
    upperBallSensor = new DigitalInput(Constants.upperBallSensorPort);
    this.register();
  }

  public enum ballMode{
    intake,
    hold,
    unloadIntake,
    unloadOutput;
  }

  public void setBallMode(ballMode mode) {
    double intakeSpeed = Robot.shuffleBoard.ballIntakeSpeed.getDouble(0);
    double flickerInSpeed = Robot.shuffleBoard.ballFlickerInSpeed.getDouble(0);
    double flickerOutIntakeSpeed = Robot.shuffleBoard.ballFluckerOutIntakeSpeed.getDouble(0);
    double beltSpeed = Robot.shuffleBoard.ballBeltsSpeed.getDouble(0);
    double beltsOutIntakeSpeed = Robot.shuffleBoard.ballBeltsOutIntakeSpeed.getDouble(0);
    double beltsOutOutputSpeed = Robot.shuffleBoard.ballBeltsOutOutputSpeed.getDouble(0);
    double outputSpeed = Robot.shuffleBoard.ballOutputSpeed.getDouble(0);

    if (mode == ballMode.intake) {
      setBallState(true, false, true, intakeSpeed, flickerInSpeed, beltSpeed, 0);
    } else if (mode == ballMode.hold) {
      setBallState(false, true, true, 0, 0, 0, 0);
    } else if (mode == ballMode.unloadIntake) {
      setBallState(false, false, true, 0, flickerOutIntakeSpeed, beltsOutIntakeSpeed, 0);
    } else if (mode == ballMode.unloadOutput) {
      setBallState(false, false, false, 0, 0, beltsOutOutputSpeed, outputSpeed);
    }
    this.mode = mode;
  }

  
  public void setBallState(boolean intakeDown, boolean lowerBlocked, boolean upperBlocked, double intakeSpeed, 
                          double flickerSpeed, double beltSpeed, double outputSpeed) {
    setIntakeDown(intakeDown);
    setLowerBlocked(lowerBlocked);
    setUpperBlocked(upperBlocked);
    setIntakeSpeed(intakeSpeed);
    // accelerateIntake(intakeTalon, intakeSpeed, .01);
    setFlickerSpeed(flickerSpeed);
    setBeltSpeed(beltSpeed);
    setOutputSpeed(outputSpeed);
  }

  // Set intake motor speed percent and outputs to dashboard.
  private void setIntakeSpeed(double x) {
    intakeTalon.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballIntakeOutput.setDouble(intakeTalon.getMotorOutputPercent());
  }

  // Set flicker motor speed percent and outputs to dashboard.
  private void setFlickerSpeed(double x) {
    flickerVictor.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballFlickerOutput.setDouble(flickerVictor.getMotorOutputPercent());
  }

  // Set belt motor speed percent and outputs to dashboard.
  private void setBeltSpeed(double x) {
    beltTalon.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballBeltsOutput.setDouble(beltTalon.getMotorOutputPercent());
  }

  // Set output motor speed percent and outputs to dashboard.
  private void setOutputSpeed(double x) {
    outputVictor.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballOutputOutput.setDouble(outputVictor.getMotorOutputPercent());
  }

  //Set intake down and outputs to dashboard.
  private void setIntakeDown(Boolean intakeDown) {
    SmartDashboard.putNumber("timeA", 0);
    SmartDashboard.putNumber("timeB", 0);
    // intakeLowerSolenoid.set(!intakeDown);
    // intakeLiftSolenoid.set(intakeDown);
    // System.out.println("BallSubsystem.setIntakeDown() intake set to ::: " + intakeDown);
    if (intakeDown) {
      // new IntakeLowerCommand(this, intakeDoubleSolenoid);
      intakeDoubleSolenoid.set(Value.kForward);
      // long timeA = (long)SmartDashboard.getNumber("timeA", 0);
      // long timeB = (long)SmartDashboard.getNumber("timeB", 0);
      long timeA = 300;
      long timeB = 75;

      new Thread(new Runnable() {
          public void run() {
            boolean runnable = true;
            while (runnable) {
              try {
                Thread.sleep(timeA);
              } catch (Exception e) {
                //TODO: handle exception
              }
              intakeDoubleSolenoid.set(Value.kReverse);
              runnable = false;
            }
          }
      }).start();

      new Thread(new Runnable() {
        public void run() {
          boolean runnable = true;
          while (runnable) {
            try {
              Thread.sleep(timeA+timeB);
            } catch (Exception e) {
              //TODO: handle exception
            }
            intakeDoubleSolenoid.set(Value.kForward);
            runnable = false;
            }
        }
      }).start();

    } else {
      intakeDoubleSolenoid.set(Value.kReverse);
    }

    // Robot.shuffleBoard.ballIntakeState.setValue(intakeLiftSolenoid.get());
  }

  //Block lower and outputs to dashboard.
  private void setLowerBlocked(Boolean blocked) {
    // lowerBlockDoubleSolenoid.set(Value.kOff);
    if (blocked) {
      lowerBlockDoubleSolenoid.set(Value.kForward);
    } else {
      lowerBlockDoubleSolenoid.set(Value.kReverse);
    }
    
    // Robot.shuffleBoard.ballLowerBlockerState.setBoolean(lowerBlockDoubleSolenoid.get());
  }

  //Block upper and outputs to dashboard.
  private void setUpperBlocked(Boolean blocked) {
    // upperBlockDoubleSolenoid.set(Value.kOff);

    if (blocked) {
      upperBlockDoubleSolenoid.set(Value.kForward);
    } else {
      upperBlockDoubleSolenoid.set(Value.kReverse);
    }

    // Robot.shuffleBoard.ballUpperBlockerState.setValue(upperBlockDoubleSolenoid.get());
  }

  //Ball sensor in
  public boolean getLowerBallSensor() {
    Robot.shuffleBoard.ballSensorInDIO.setBoolean(lowerBallSensor.get());
    return lowerBallSensor.get();
  }

  //Ball sensor out
  public boolean getUpperBallSensor() {
    Robot.shuffleBoard.ballSensorOutDIO.setBoolean(upperBallSensor.get());
    return upperBallSensor.get();
  }

  // Function which handles ball counting logic.
  private void ballCounter() {
    if (getLowerBallSensor() && !lowerBallSensorBlocked) {
      if (mode == ballMode.intake) {
        storedBalls ++;
      } else if (mode == ballMode.unloadIntake) {
        storedBalls --;
      }
      lowerBallSensorBlocked = true;
    } else if (!getLowerBallSensor() && lowerBallSensorBlocked) {
      lowerBallSensorBlocked = false;
    }

    if (getUpperBallSensor() && !upperBallSensorBlocked) {
      upperBallSensorBlocked = true;
    } else if (!getUpperBallSensor() && upperBallSensorBlocked) {
      storedBalls --;
      upperBallSensorBlocked = false;
    }

    // stop once 5 balls
    if (storedBalls >= maxStoredBalls && mode == ballMode.intake) {
      setBallMode(ballMode.hold);
    }
  }

  // private void accelerateIntake(WPI_TalonSRX talonSRX, double setPercentage, double acceleration) {
  //   double tolerance = acceleration * 1.5;
  //   Looper accelerator;
  //   Loop accelerate = new Loop(){
  //       @Override public void onStart() {
  //       }
  //       @Override public void onLoop() {
  //         double currentPercent = talonSRX.get();
  //         if (Math.abs(currentPercent - setPercentage) < tolerance) {
  //           looper.stop();
  //         }
  //         if (currentPercent < setPercentage) {
  //           setIntakeSpeed(currentPercent + acceleration);
  //         } else if (currentPercent > setPercentage) {
  //           setIntakeSpeed(currentPercent - acceleration);
  //         }
  //       }
  //       @Override public void onStop() {
  //         setIntakeSpeed(setPercentage);
  //       }
  //     };
  //     accelerator = new Looper(accelerate, 50);
  //     accelerator.start();
  // }
  

  @Override
  public void periodic() {
    ballCounter();
    Robot.shuffleBoard.ballSensorInDIO.setBoolean(lowerBallSensor.get());
    Robot.shuffleBoard.ballSensorOutDIO.setBoolean(upperBallSensor.get());
    Robot.shuffleBoard.ballCount.setNumber(storedBalls);
  }
}