package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX winch, elevator;

  private final double maxVelocity = 100;
  private final int encoderTicks = 4096;

  private final int winchTalonPort = 4, 
                    elevatorTalonPort = 5;

  public ClimbSubsystem() {
    winch = new WPI_TalonSRX(winchTalonPort);
    elevator = new WPI_TalonSRX(elevatorTalonPort);

    elevator.setSelectedSensorPosition(0);
    elevator.configNeutralDeadband(0.001);
    elevator.setNeutralMode(NeutralMode.Brake);
    elevator.setSensorPhase(true);
    elevator.setInverted(false);

    elevator.config_kP(0, Robot.shuffleBoard.climbElevatorPidP.getDouble(0));
    elevator.config_kI(0, Robot.shuffleBoard.climbElevatorPidI.getDouble(0));
    elevator.config_kD(0, Robot.shuffleBoard.climbElevatorPidD.getDouble(0));
    elevator.config_kF(0, Robot.shuffleBoard.climbElevatorPidF.getDouble(0));

    elevator.configForwardSoftLimitThreshold(1300);
		elevator.configForwardSoftLimitEnable(false);
		elevator.configReverseSoftLimitThreshold(0);
		elevator.configReverseSoftLimitEnable(false);
  }

  public void resetEncoder() {
    elevator.set(ControlMode.PercentOutput, -.2);
    while(!elevator.getSensorCollection().isRevLimitSwitchClosed()) {}
    elevator.stopMotor();
    elevator.setSelectedSensorPosition(0);
  }

  public double getElevatorVelocityPercent() {
    return elevator.getSelectedSensorVelocity(0) / maxVelocity;
  }

  public double getElevatorHeight() {
    return elevator.getSelectedSensorPosition() / encoderTicks;
  }

  public void setWinch(double percent) {
    winch.set(ControlMode.PercentOutput, percent);
  }

  public double getWinch() {
    return winch.get();
  }

  public void stopWinch() {
    winch.set(ControlMode.PercentOutput, 0);
    winch.stopMotor();
  }

  public void setElevatorVelocity(double velocity) {
    elevator.set(ControlMode.Velocity, velocity);
  }

  public void stopElevator() {
    elevator.set(ControlMode.PercentOutput, 0);
    elevator.stopMotor();
  }

  public boolean elevatorLowerLimit() {
    return false;
  }

  public boolean elevatorUpperLimit() {
    return false;
  }
}
