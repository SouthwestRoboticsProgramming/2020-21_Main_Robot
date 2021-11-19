package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.Servo;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX winch;
  private final WPI_VictorSPX elevator;
  private final Servo lockerServo;

  public ClimbSubsystem() {
    winch = new WPI_TalonSRX(Constants.winchTalonPort);
    elevator = new WPI_VictorSPX(Constants.elevatorTalonPort);
    lockerServo = new Servo(0);
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
    // System.out.println("climbOutput = " + percent);
    elevator.set(ControlMode.PercentOutput, percent);
    Robot.shuffleBoard.climbElevatorOutput.setDouble(elevator.getMotorOutputPercent());
  }

  public void setLock(boolean lock) {
    if (lock) {
      lockerServo.setAngle(0); //Degrees (Could be wrong, in testing)
    } else {
      lockerServo.setAngle(90); //Degrees (Could be wrong, in testing)
    }
  }
}