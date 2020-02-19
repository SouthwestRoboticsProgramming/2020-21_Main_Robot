/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveWithLimelightToHp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveWithLimilightToHP extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private PID pid;

  public DriveWithLimilightToHP(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    // addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveWithLimilightToHP.initialize()");
    double[] tPid = new double[] {Robot.shuffleBoard.driveTurnPidP.getDouble(0),
      Robot.shuffleBoard.driveTurnPidI.getDouble(0),
      Robot.shuffleBoard.driveTurnPidD.getDouble(0)};
    pid = new PID(tPid[0], tPid[1], tPid[2]);
    pid.setSetPoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = pid.getOutput(-Robot.limelight.getX());
    System.out.println("limelightX = " + Robot.limelight.getX());
    double drive = 0;
    // double turn = 0;
    // double drive = Robot.limelight.getArea()/300;
    driveTrainSubsystem.driveMotors(turn + drive, -turn + drive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.driveMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
