package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX winch, elevator;

  public ClimbSubsystem() {
    winch = new WPI_TalonSRX(Constants.winchTalonPort);
    elevator = new WPI_TalonSRX(Constants.elevatorTalonPort);
    winch.setInverted(true);
    winch.setNeutralMode(NeutralMode.Brake);

    elevator.configNeutralDeadband(0.001);
    elevator.setNeutralMode(NeutralMode.Brake);
    elevator.setInverted(false);

  }

  public void setWinch(double percent) {
    winch.set(ControlMode.PercentOutput, percent);
    Robot.shuffleBoard.climbWinchOutput.setDouble(winch.getMotorOutputPercent());
  }

  public void setElevator(double percent) {
    elevator.set(ControlMode.PercentOutput, percent);
    Robot.shuffleBoard.climbElevatorOutput.setDouble(elevator.getMotorOutputPercent());
  }
}