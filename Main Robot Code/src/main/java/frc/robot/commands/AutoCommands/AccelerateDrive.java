/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.WheelSide;

public class AccelerateDrive extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double start;
  private double end;
  private double lastSpeed;
  private double acceleration;
  private boolean finished = false;
  private double tolerance;
  private WheelSide wheel;

  public AccelerateDrive(DriveTrainSubsystem driveTrainSubsystem, double start, double end, WheelSide wheel) {
    // addRequirements(driveTrainSubsystem);
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.start = start;
    this.end = end;
    this.wheel = wheel;
    System.out.println("AccelerateDrive.enclosing_method()");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveMotors(0, 0);
    lastSpeed = start;
    this.acceleration = Robot.shuffleBoard.autoTuneAcceleration.getDouble(0);
    this.tolerance = acceleration * 1.5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newSpeed;
    if (start < end) {
      newSpeed = lastSpeed + acceleration;
    } else {
      newSpeed = lastSpeed - acceleration;
    }
    driveMotors(newSpeed, newSpeed);
    lastSpeed = newSpeed;
    if (Math.abs(end-newSpeed) < tolerance) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveMotors(end, end);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }


  private void driveMotors(double left, double right) {
    if (wheel == WheelSide.left) {
      driveTrainSubsystem.driveMotors(left, 0);
    } else if (wheel == WheelSide.right) {
      driveTrainSubsystem.driveMotors(0, right);
    } else if (wheel == WheelSide.both) {
      driveTrainSubsystem.driveMotors(left, right);
    }
    
  }
}
