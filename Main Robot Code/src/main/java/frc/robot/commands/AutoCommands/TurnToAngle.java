/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem.WheelSide;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TurnToAngle extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double angle;
  private WheelSide wheel;
  private PID pid;
  private boolean finished;

  public TurnToAngle(DriveTrainSubsystem driveTrainSubsystem, double angle, WheelSide wheel) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.angle = angle;
    this.wheel = wheel;
    // addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    double[] tPid = driveTrainSubsystem.getTurnPid();
    System.out.println("switch = " + Math.abs(angle - Robot.gyro.getGyroAngleZ()));
    if (Math.abs(angle - Robot.gyro.getGyroAngleZ()) > 40) {
      tPid[1] = 0;
    }
    pid = new PID(tPid[0], tPid[1], tPid[2]);
    pid.setSetPoint(angle);
    finished = false;
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double output = -pid.getOutput(Robot.gyro.getGyroAngleZ());
    if (output > 0) {
      output += Robot.shuffleBoard.driveTurnPidF.getDouble(0);
    } else {
      output -= Robot.shuffleBoard.driveTurnPidF.getDouble(0);
    }
    System.out.println("TurnToAngle.execute() out = " + pid.getError());
    if (wheel == WheelSide.left) {
      driveTrainSubsystem.driveMotors(output, 0);
    } else if (wheel == WheelSide.right) {
      driveTrainSubsystem.driveMotors(0, -output);
    } else {
      driveTrainSubsystem.driveMotors(.5*output, -.5*output);
    }

    
    if (Math.abs(pid.getError()) <1) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.driveMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
