/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Counter;

public class BallSubsystem extends SubsystemBase {
  /**
   * The ball subsystem includes the intake motor and solenoids, the belt motor, and the output motor and solenoid.
   * It also includes two sensors used for counting stored balls.
   * The ballCounters are there to ensure that no balls are missed by the sensors.
   */

  private WPI_TalonSRX intakeTalon, ballFlickerTalon beltTalon, outputTalon;
  private Solenoid lowerIntakeSolenoid, liftIntakeSolenoid, blockBlockSolenoid, blockUnBlockSolenoid;
  private DigitalInput ballSensorIn, ballSensorOut;

  private int intakeTalonPort = 0,
              ballFlickerTalonPort = 0,
              beltTalonPort = 0,
              outputTalonPort = 0;

  private int lowerIntakeSolenoidPort = 0,
              liftIntakeSolenoidPort = 0,
              blockBlockSolenoidPort = 0,
              blockUnBlockSolenoidPort = 0;

  private int ballSensorInPort = 0,
              ballSensorOutPort = 0;

  private int storedBalls = 3;
  private final int maxStoredBalls = 5;

  private boolean ballSensorInBlocked = false,
                  ballSensorOutBlocked = false;

  public BallSubsystem() {
    intakeTalon = new WPI_TalonSRX(intakeTalonPort);
    ballFlickerTalon = new WPI_TalonSRX(ballFlickerTalonPort);
    beltTalon = new WPI_TalonSRX(beltTalonPort);
    outputTalon = new WPI_TalonSRX(outputTalonPort);
  
    intakeTalon.setNeutralMode(NeutralMode.Brake);
    ballFlickerTalon.setNeutralMode(NeutralMode.Brake);
    beltTalon.setNeutralMode(NeutralMode.Brake);
    outputTalon.setNeutralMode(NeutralMode.Brake);

    lowerIntakeSolenoid = new Solenoid(lowerIntakeSolenoidPort);
    liftIntakeSolenoid = new Solenoid(liftIntakeSolenoidPort);
    blockBlockSolenoid = new Solenoid(blockBlockSolenoidPort);
    blockUnBlockSolenoid = new Solenoid(blockUnBlockSolenoidPort);

    ballSensorIn = new DigitalInput(ballSensorInPort);
    ballSensorOut = new DigitalInput(ballSensorOutPort);
  }

  //Intake motor
  public void setIntake(double x) {
    intakeTalon.set(x);
  }

  // flicker motor
  public void setFlicker(double x) {
    flickerTalon.set(x);
  }

  //Belt motor
  public void setBelt(double x) {
    beltTalon.set(x);

  }

  //Output motor
  public void setOutput(double x) {
    outputTalon.set(x);
  }

  //Intake solenoids

  //Output solenoids

  //Ball sensor in
  public boolean getBallSensorIn() {
    return ballSensorIn.get();
  }

  //Ball sensor out
  public boolean getBallSensorOut() {
    return ballSensorOut.get();
  }

  public void periodic() {
  if (ballSystemIsForwards) {

  }
  }
}
