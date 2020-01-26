package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import frc.robot.Robot;

//TODO: Clarify lift, lower, block, and unblock in solenoid names. Difficult to understand what does what.

/*
* Ball subsystem includes all motors, solenoids, and sensors assosiated with picking up and storing balls.
*/
public class BallSubsystem extends SubsystemBase {
  private final WPI_TalonSRX intakeTalon;
  private final WPI_VictorSPX ballFlickerVictor, beltVictor, outputVictor;
  private final Solenoid lowerIntakeSolenoid, liftIntakeSolenoid, lowerBlockSolenoid, lowerUnBlockSolenoid, upperBlockSolenoid, upperUnBlockSolenoid;
  private final DigitalInput ballSensorIn, ballSensorOut;

  private final int lowerIntakeSolenoidPort = 0,
              liftIntakeSolenoidPort = 1,
              lowerBlockSolenoidPort = 4,
              lowerUnBlockSolenoidPort = 5,
              upperBlockSolenoidPort = 2,
              upperUnBlockSolenoidPort = 3;

  private final int ballSensorInPort = 0,
              ballSensorOutPort = 1;

  private int storedBalls = 3;
  private final int maxStoredBalls = 5;
  private ballMode mode;

  private boolean ballSensorInBlocked = false,
                  ballSensorOutBlocked = false;

  public BallSubsystem() {
    intakeTalon = new WPI_TalonSRX(Constants.intakeTalonPort);
    ballFlickerVictor = new WPI_VictorSPX(Constants.ballFlickerTalonPort);
    beltVictor = new WPI_VictorSPX(Constants.beltTalonPort);
    outputVictor = new WPI_VictorSPX(Constants.outputTalonPort);
  
    intakeTalon.setNeutralMode(NeutralMode.Brake);
    ballFlickerVictor.setNeutralMode(NeutralMode.Brake);
    beltVictor.setNeutralMode(NeutralMode.Brake);
    outputVictor.setNeutralMode(NeutralMode.Brake);

    lowerIntakeSolenoid = new Solenoid(lowerIntakeSolenoidPort);
    liftIntakeSolenoid = new Solenoid(liftIntakeSolenoidPort);
    lowerBlockSolenoid = new Solenoid(lowerBlockSolenoidPort);
    lowerUnBlockSolenoid = new Solenoid(lowerUnBlockSolenoidPort);
    upperBlockSolenoid = new Solenoid(upperBlockSolenoidPort);
    upperUnBlockSolenoid = new Solenoid(upperUnBlockSolenoidPort);

    ballSensorIn = new DigitalInput(ballSensorInPort);
    ballSensorOut = new DigitalInput(ballSensorOutPort);
  }

  // ball state
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
    double beltsSpeed = Robot.shuffleBoard.ballBeltsSpeed.getDouble(0);
      double beltsOutIntakeSpeed = Robot.shuffleBoard.ballBeltsOutIntakeSpeed.getDouble(0);
      double beltsOutOutputSpeed = Robot.shuffleBoard.ballBeltsOutOutputSpeed.getDouble(0);
    double outputSpeed = Robot.shuffleBoard.ballOutputSpeed.getDouble(0);

    if (mode == ballMode.intake) {
      setBallState(true, false, true, intakeSpeed, flickerInSpeed, beltsSpeed, outputSpeed);
    } else if (mode == ballMode.hold) {
      setBallState(false, true, true, 0, 0, 0, 0);
    } else if (mode == ballMode.unloadIntake) {
      setBallState(false, false, true, 0, flickerOutIntakeSpeed, beltsOutIntakeSpeed, 0);
    } else if (mode == ballMode.unloadOutput) {
      setBallState(false, false, false, 0, 0, beltsOutOutputSpeed, outputSpeed);
    }
    this.mode = mode;
  }

  
  private void setBallState(boolean intakeLowered, boolean lowerBlocked, boolean upperBlocked, double ballIntakeSpeed, 
                          double ballFlickerSpeed, double belts, double output) {
    setIntake(intakeLowered);
    setLowerBlocker(lowerBlocked);
    setUpperBlocker(upperBlocked);
    setIntake(ballIntakeSpeed);
    setBallFlicker(ballFlickerSpeed);
    setBelt(belts);
    setOutput(output);
  }

  //Intake motor
  private void setIntake(double x) {
    intakeTalon.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballIntakeOutput.setDouble(intakeTalon.getMotorOutputPercent());
  }

  // ball flicker motor
  private void setBallFlicker(double x) {
    ballFlickerVictor.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballFlickerOutput.setDouble(ballFlickerVictor.getMotorOutputPercent());
  }

  //Belt motor
  private void setBelt(double x) {
    beltVictor.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballBeltsOutput.setDouble(beltVictor.getMotorOutputPercent());
  }

  //Output motor
  private void setOutput(double x) {
    outputVictor.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.ballOutputOutput.setDouble(outputVictor.getMotorOutputPercent());
  }

  //Intake solenoids
  private void setIntake(Boolean intake) {
    liftIntakeSolenoid.set(!intake);
    lowerIntakeSolenoid.set(intake);
    Robot.shuffleBoard.ballLiftIntakeSolenoid.setBoolean(liftIntakeSolenoid.get());
    Robot.shuffleBoard.ballLowerIntakeSolenoid.setBoolean(lowerIntakeSolenoid.get());
  }

  //Blocker solenoids
  private void setLowerBlocker(Boolean blocked) {
    lowerBlockSolenoid.set(blocked);
    lowerUnBlockSolenoid.set(!blocked);
    Robot.shuffleBoard.ballLowerBlockSolenoid.setBoolean(lowerBlockSolenoid.get());
    Robot.shuffleBoard.ballLowerUnBlockSolenoid.setBoolean(lowerUnBlockSolenoid.get());
  }

  private void setUpperBlocker(Boolean blocked) {
    upperBlockSolenoid.set(blocked);
    upperUnBlockSolenoid.set(!blocked);
    Robot.shuffleBoard.ballUpperBlockSolenoid.setBoolean(upperBlockSolenoid.get());
    Robot.shuffleBoard.ballUpperUnBlockSolenoid.setBoolean(upperUnBlockSolenoid.get());
  }

  //Ball sensor in
  public boolean getBallSensorIn() {
    Robot.shuffleBoard.ballSensorInDIO.setBoolean(ballSensorIn.get());
    return ballSensorIn.get();
  }

  //Ball sensor out
  public boolean getBallSensorOut() {
    Robot.shuffleBoard.ballSensorOutDIO.setBoolean(ballSensorOut.get());
    return ballSensorOut.get();
  }

  @Override
  public void periodic() {
    // countBalls
    if (getBallSensorIn() && !ballSensorInBlocked) {
      if (mode == ballMode.intake) {
        storedBalls ++;
      } else if (mode == ballMode.unloadIntake) {
        storedBalls --;
      }
      ballSensorInBlocked = true;
    } else if (!getBallSensorIn() && ballSensorInBlocked) {
      ballSensorInBlocked = false;
    }

    if (getBallSensorOut() && !ballSensorOutBlocked) {
      storedBalls --;
      ballSensorOutBlocked = true;
    } else if (!getBallSensorOut() && ballSensorOutBlocked) {
      ballSensorOutBlocked = false;
    }

    // stop once 5 balls
    if (storedBalls >= maxStoredBalls && mode == ballMode.intake) {
      setBallMode(ballMode.hold);
    }
  }
}