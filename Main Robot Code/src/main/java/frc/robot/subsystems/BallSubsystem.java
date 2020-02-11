package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Looper.Loop;
import frc.lib.Looper.Looper;
import frc.robot.Constants;
import frc.robot.Robot;

//TODO: Clarify lift, lower, block, and unblock in solenoid names. Difficult to understand what does what.

/*
* Ball subsystem includes all motors, solenoids, and sensors assosiated with picking up and storing balls.
  Note that "lower" refers to the part of the belt system near the intake, while "upper" refers to the other end.
*/
public class BallSubsystem extends SubsystemBase {
  private final WPI_TalonSRX intakeTalon;
  private final WPI_VictorSPX flickerVictor, beltVictor, outputVictor;
  private final Solenoid intakeLowerSolenoid, intakeLiftSolenoid;
  private final DoubleSolenoid lowerBlockDoubleSolenoid, upperBlockDoubleSolenoid;
  private final DigitalInput lowerBallSensor, upperBallSensor;

  private int storedBalls = 3;
  private final int maxStoredBalls = 5;
  private ballMode mode;
  private Looper looper;

  private boolean lowerBallSensorBlocked = false,
                  upperBallSensorBlocked = false;

  public BallSubsystem() {
    intakeTalon = new WPI_TalonSRX(Constants.intakeTalonPort);
    flickerVictor = new WPI_VictorSPX(Constants.flickerVictorPort);
    beltVictor = new WPI_VictorSPX(Constants.beltVictorPort);
    outputVictor = new WPI_VictorSPX(Constants.outputVictorPort);
  
    intakeTalon.setNeutralMode(NeutralMode.Brake);
    flickerVictor.setNeutralMode(NeutralMode.Brake);
    beltVictor.setNeutralMode(NeutralMode.Brake);
    outputVictor.setNeutralMode(NeutralMode.Brake);

    intakeLowerSolenoid = new Solenoid(Constants.lowerIntakeSolenoidPort);
    intakeLiftSolenoid = new Solenoid(Constants.liftIntakeSolenoidPort);
    lowerBlockDoubleSolenoid = new DoubleSolenoid(Constants.closeLowerSolenoidPort, Constants.openLowerSolenoidPort);
    upperBlockDoubleSolenoid = new DoubleSolenoid(Constants.closeUpperSolenoidPort, Constants.openUpperSolenoidPort);

    lowerBallSensor = new DigitalInput(Constants.lowerBallSensorPort);
    upperBallSensor = new DigitalInput(Constants.upperBallSensorPort);
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
      setBallState(true, false, true, intakeSpeed, flickerInSpeed, beltSpeed, outputSpeed);
    } else if (mode == ballMode.hold) {
      setBallState(false, true, true, 0, 0, 0, 0);
    } else if (mode == ballMode.unloadIntake) {
      setBallState(false, false, true, 0, flickerOutIntakeSpeed, beltsOutIntakeSpeed, 0);
    } else if (mode == ballMode.unloadOutput) {
      setBallState(false, false, false, 0, 0, beltsOutOutputSpeed, outputSpeed);
    }
    this.mode = mode;
  }

  
  private void setBallState(boolean intakeDown, boolean lowerBlocked, boolean upperBlocked, double intakeSpeed, 
                          double flickerSpeed, double beltSpeed, double outputSpeed) {
    setIntakeDown(intakeDown);
    setLowerBlocked(lowerBlocked);
    setUpperBlocked(upperBlocked);
    setIntakeSpeed(intakeSpeed);
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
    beltVictor.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballBeltsOutput.setDouble(beltVictor.getMotorOutputPercent());
  }

  // Set output motor speed percent and outputs to dashboard.
  private void setOutputSpeed(double x) {
    outputVictor.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballOutputOutput.setDouble(outputVictor.getMotorOutputPercent());
  }

  //Set intake down and outputs to dashboard.
  private void setIntakeDown(Boolean intakeDown) {
    if (intakeDown) {
      final int timeToPushForward = 10;
      final int timeBeforeStopping = 10;
      final int timeBeforeHolingIntakeDown = 10;
      long currentTime = System.currentTimeMillis();

      //Push intake forward
      Loop pushForward = new Loop(){
        @Override public void onStart() {
          intakeLowerSolenoid.set(true);
          intakeLiftSolenoid.set(false);
        }
        @Override public void onLoop() {
          if (currentTime + timeToPushForward >= System.currentTimeMillis()) {
            looper.stop();
          }
        }
        @Override public void onStop() {
          intakeLowerSolenoid.set(false);
            intakeLiftSolenoid.set(false);
        }
      };
      looper = new Looper(pushForward, 50);
      looper.start();

      
      Loop waitToStop = new Loop(){
        @Override public void onStart() {
        }
        @Override public void onLoop() {
          if (currentTime + timeToPushForward + timeBeforeStopping >= System.currentTimeMillis()) {
            intakeLowerSolenoid.set(false);
            intakeLiftSolenoid.set(true);
            looper.stop();
          }
        }
        @Override public void onStop() {
        }
      };
      looper = new Looper(waitToStop, 50);
      looper.start();

      Loop holdDown = new Loop(){
        @Override public void onStart() {
        }
        @Override public void onLoop() {
          if (currentTime + timeToPushForward + timeBeforeStopping + timeBeforeHolingIntakeDown >= System.currentTimeMillis()) {
            intakeLowerSolenoid.set(true);
            intakeLiftSolenoid.set(false);
            looper.stop();
          }
        }
        @Override public void onStop() {
        }
      };
      looper = new Looper(waitToStop, 50);
      looper.start();

    } else {
      intakeLowerSolenoid.set(false);
      intakeLiftSolenoid.set(true);
    }

    Robot.shuffleBoard.ballIntakeState.setValue(intakeLiftSolenoid.get());
  }

  //Block lower and outputs to dashboard.
  private void setLowerBlocked(Boolean blocked) {
    if (blocked) {
      lowerBlockDoubleSolenoid.set(Value.kForward);
    } else {
      lowerBlockDoubleSolenoid.set(Value.kReverse);
    }
    
    Robot.shuffleBoard.ballLowerBlockerState.setValue(lowerBlockDoubleSolenoid.get());
  }

  //Block upper and outputs to dashboard.
  private void setUpperBlocked(Boolean blocked) {
    if (blocked) {
      upperBlockDoubleSolenoid.set(Value.kForward);
    } else {
      upperBlockDoubleSolenoid.set(Value.kReverse);
    }

    Robot.shuffleBoard.ballUpperBlockerState.setValue(upperBlockDoubleSolenoid.get());
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
      storedBalls --;
      upperBallSensorBlocked = true;
    } else if (!getUpperBallSensor() && upperBallSensorBlocked) {
      upperBallSensorBlocked = false;
    }

    // stop once 5 balls
    if (storedBalls >= maxStoredBalls && mode == ballMode.intake) {
      setBallMode(ballMode.hold);
    }
  }
  

  @Override
  public void periodic() {
    ballCounter();
  }
}