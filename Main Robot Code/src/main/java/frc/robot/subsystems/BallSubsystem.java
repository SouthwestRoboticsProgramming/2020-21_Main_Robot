package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final Solenoid intakeLowerSolenoid, intakeLiftSolenoid;
  private final DigitalInput lowerBallSensor, upperBallSensor;
  private final Encoder beltEncoder;

  //Balls currently in possesion of bot.
  private int ballsOnRobot = 0;

  //Balls in the upper part of belt system.
  private int ballsHeld = 0;
  private final int maxStoredBalls = 5;
  private ballMode mode;

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

    beltEncoder = new Encoder(Constants.beltEncoderAPort, Constants.beltEncoderBPort);
    beltEncoder.setReverseDirection(true);
    beltEncoder.reset();
  
    intakeTalon.setNeutralMode(NeutralMode.Brake);
    flickerVictor.setNeutralMode(NeutralMode.Brake);
    beltTalon.setNeutralMode(NeutralMode.Brake);
    outputVictor.setNeutralMode(NeutralMode.Brake);
    intakeLowerSolenoid = new Solenoid(37, Constants.lowerIntakeSolenoidPort);
    intakeLiftSolenoid = new Solenoid(37, Constants.liftIntakeSolenoidPort);

    lowerBallSensor = new DigitalInput(Constants.lowerBallSensorPort);
    upperBallSensor = new DigitalInput(Constants.upperBallSensorPort);
    this.register();

  }

  public enum ballMode{
    intake,
    intakeNoDrop,
    hold,
    unloadIntake,
    unloadOutput,
    pushBalls;
  }

  public void setBallMode(ballMode mode) {
    double intakeSpeed = Robot.shuffleBoard.ballIntakeSpeed.getDouble(0);
    double pushSpeed = Robot.shuffleBoard.ballPushSpeed.getDouble(0);
    double flickerInSpeed = Robot.shuffleBoard.ballFlickerInSpeed.getDouble(0);
    double flickerOutIntakeSpeed = Robot.shuffleBoard.ballFluckerOutIntakeSpeed.getDouble(0);
    double beltSpeed = Robot.shuffleBoard.ballBeltsSpeed.getDouble(0);
    double beltsOutIntakeSpeed = Robot.shuffleBoard.ballBeltsOutIntakeSpeed.getDouble(0);
    double beltsOutOutputSpeed = Robot.shuffleBoard.ballBeltsOutOutputSpeed.getDouble(0);
    double outputSpeed = Robot.shuffleBoard.ballOutputSpeed.getDouble(0);
    double outputHoldSpeed = Robot.shuffleBoard.ballPushHoldSpeed.getDouble(0);

    if (mode == ballMode.intake) {
      setBallState(true, false, true, intakeSpeed, flickerInSpeed, 0, outputHoldSpeed);
    } else if (mode == ballMode.intakeNoDrop) {
      setBallState(false, false, true, 0, flickerInSpeed, 0, outputHoldSpeed);
    } else if (mode == ballMode.hold) {
      setBallState(false, true, true, 0, 0, 0, outputHoldSpeed);
    } else if (mode == ballMode.unloadIntake) {
      setBallState(false, false, true, 0, flickerOutIntakeSpeed, beltsOutIntakeSpeed, 0);
    } else if (mode == ballMode.unloadOutput) {
      setBallState(false, false, false, 0, .1, beltsOutOutputSpeed, outputSpeed);
    } else if (mode == ballMode.pushBalls) {
      setBallState(true, true, true, pushSpeed, 0, 0, outputHoldSpeed);
    }
    this.mode = mode;
  }

  public void setBallState(boolean intakeDown, boolean lowerBlocked, boolean upperBlocked, double intakeSpeed, 
                          double flickerSpeed, double beltSpeed, double outputSpeed) {
    setIntakeDown(intakeDown);
    setIntakeSpeed(intakeSpeed);
    setFlickerSpeed(flickerSpeed);
    setBeltSpeed(beltSpeed);
    setOutputSpeed(outputSpeed);
  }

  // Sets ballsHeld and ballsOnRobot to count
  public void setBallCount(int count) {
    ballsHeld = count;
    ballsOnRobot = count;
  }

  // Set intake motor speed percent and outputs to dashboard.
  private void setIntakeSpeed(double x) {
    if (x == 0) {
      intakeTalon.set(ControlMode.PercentOutput, x);
    } else {
      //ms before intake changes to nonzero speed
      long intakeSpeedDelay = 1000;
        new Thread(new Runnable() {
            public void run() {
              boolean runnable = true;
              
              while (runnable) {
                try {
                  Thread.sleep(intakeSpeedDelay);
                } catch (Exception e) {
                  //TODO: handle exception
                }
                //Prevents intake from spinning when intake is up
                if (intakeLowerSolenoid.get() == true) {
                  intakeTalon.set(ControlMode.PercentOutput, x);
                }
                runnable = false;
              }
            }
        }).start();
      }
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
    if (intakeDown) {
      intakeLowerSolenoid.set(true);
      intakeLiftSolenoid.set(true);
    } else {
      intakeLowerSolenoid.set(false);
      intakeLiftSolenoid.set(false);
    }

    // Robot.shuffleBoard.ballIntakeState.setValue(intakeLiftSolenoid.get());
  }


  //Ball sensor in
  public boolean getLowerBallSensor() {
    Robot.shuffleBoard.ballSensorInDIO.setBoolean(lowerBallSensor.get());
    boolean low = !lowerBallSensor.get();
    boolean high = !upperBallSensor.get();
    return  low && high;
  }

  //Ball sensor out
  public boolean getUpperBallSensor() {
    Robot.shuffleBoard.ballSensorOutDIO.setBoolean(upperBallSensor.get());
    return upperBallSensor.get();
  }

  @Override
  public void periodic() {
    // System.out.println("beltEncoder=" + beltEncoder.get());
    ballCounter();
    // ballSpacer();
    if (Robot.robotContainer.getPOV() == 270) {
      setBallMode(ballMode.unloadIntake);
      ballsOnRobot = 0;
    }
    // checkForJam();
    Robot.shuffleBoard.ballSensorInDIO.setBoolean(lowerBallSensor.get());
    Robot.shuffleBoard.ballSensorOutDIO.setBoolean(upperBallSensor.get());
    Robot.shuffleBoard.ballCount.setNumber(ballsOnRobot);    
  }

  private void checkForJam() {
    if (Robot.pdp.PDPBeltAmp() > Robot.shuffleBoard.ballBeltMaxAmp.getDouble(0)) {
      beltTalon.set(ControlMode.PercentOutput, 0);
      setBallMode(ballMode.hold);
    }
  }

  private void ballSpacer() {
    boolean intake = ballMode.intake == mode || ballMode.intakeNoDrop == mode;
    if(intake && ballsHeld < ballsOnRobot) {      
      new Thread(new Runnable() {
        public void run() {
          boolean runnable = true;
          while (runnable) {
            try {
              if (ballsOnRobot < maxStoredBalls) {
                Thread.sleep((long)Robot.shuffleBoard.ballSpacingWait.getDouble(1000));
              } else {
                Thread.sleep((long)Robot.shuffleBoard.ball5thSpacingWait.getDouble(1000));
              }
            } catch (Exception e) {
              //TODO: handle exception
            }
            runnable = false;
            }
        }
      }).start();
      if (intake && ballsOnRobot < 5) {
        beltTalon.set(ControlMode.PercentOutput, Robot.shuffleBoard.ballBeltsSpeed.getDouble(0));
      }
      intakeTalon.set(ControlMode.PercentOutput, 0);
      new Thread(new Runnable() {
        public void run() {
          boolean runnable = true;
          while (runnable) {
            try {
              if (ballsOnRobot < maxStoredBalls) {
                Thread.sleep((long)Robot.shuffleBoard.ballSpacingMove.getDouble(1000));
              } else {
                Thread.sleep((long)Robot.shuffleBoard.ball5thSpacingMove.getDouble(1000));
              }
              
            } catch (Exception e) {
              //TODO: handle exception
            }
            beltTalon.set(ControlMode.PercentOutput, 0);
            if (mode == ballMode.intake) {
              intakeTalon.set(ControlMode.PercentOutput, 100);
            }
            runnable = false;
            //TODO: enforce max ball count
            // stop intaking if 5 balls after belts stop moving
            if (ballsOnRobot >= maxStoredBalls) {
              setBallMode(ballMode.hold);
            }
            
            }
        }
      }).start();

      // new Thread(new Runnable() {
      //     public void run() {
      //       boolean runnable = true;
      //       while (runnable) {
      //         int encoderSet;
      //         if (ballsOnRobot < maxStoredBalls) {
      //          encoderSet = beltEncoder.get() + (int)Robot.shuffleBoard.ballSpacingMove.getDouble(0);
      //         } else {
      //           encoderSet = beltEncoder.get() + (int)Robot.shuffleBoard.ball5thSpacingMove.getDouble(0);
      //         }
      //         while (beltEncoder.get() < encoderSet && !Robot.robotContainer.voidTask()) {
      //           try {
      //             Thread.sleep(50);
      //           } catch (Exception e) {}
      //         }
      //         beltTalon.set(ControlMode.PercentOutput, 0);
      //         if (mode == ballMode.intake) {
      //           intakeTalon.set(ControlMode.PercentOutput, 100);
      //         }
      //         runnable = false;
      //         // stop intaking if 5 balls after belts stop moving
      //         if (ballsOnRobot >= maxStoredBalls) {
      //           setBallMode(ballMode.hold);
      //         }
              
      //         }
      //     }
      //   }).start();

      ballsHeld ++;
    }
  }

  // Function which handles ball counting logic.
  private void ballCounter() {
    boolean intake = ballMode.intake == mode || ballMode.intakeNoDrop == mode;
    if (getLowerBallSensor() && !lowerBallSensorBlocked) {
      if (mode == ballMode.intake || mode == ballMode.intakeNoDrop) {
        ballsOnRobot ++;
       
        if (intake && ballsOnRobot < 5) {
          beltTalon.set(ControlMode.PercentOutput, Robot.shuffleBoard.ballBeltsSpeed.getDouble(0));
        } else {
          flickerVictor.set(ControlMode.PercentOutput, 0);
          setBallMode(ballMode.hold);
        }

      } else if (mode == ballMode.unloadIntake) {
        // ballsOnRobot --;
        // ballsHeld --;
      }
      lowerBallSensorBlocked = true;
    } else if (!getLowerBallSensor() && lowerBallSensorBlocked) {
      if (intake && ballsOnRobot < 5) {
        stopBeltsAfter((int)Robot.shuffleBoard.ballSpacingMove.getDouble(0));
      }
      
      lowerBallSensorBlocked = false;
    }

    if (getUpperBallSensor() && !upperBallSensorBlocked) {
      upperBallSensorBlocked = true;
    } else if (!getUpperBallSensor() && upperBallSensorBlocked) {
      // ballsOnRobot --;
      // ballsHeld --;
      upperBallSensorBlocked = false;
    }   
  }

  private void stopBeltsAfter(int distence) {
    new Thread(new Runnable() {
      public void run() {
        boolean runnable = true;
        int start = beltEncoder.get();
        while (runnable) {
          try {
              Thread.sleep(50);
          } catch (Exception e) {
            //TODO: handle exception
          }
          if (beltEncoder.get() > start + distence) {
            beltTalon.set(ControlMode.PercentOutput, 0);
            runnable = false;
          }
        }
      }
    }).start();
  }
}