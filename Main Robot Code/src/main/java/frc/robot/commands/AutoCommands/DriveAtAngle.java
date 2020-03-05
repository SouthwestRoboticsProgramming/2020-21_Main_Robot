/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.WheelSide;

public class DriveAtAngle extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double ft;
  private double angle;
  private double speed;
  private boolean finished;
  private double lastLeftFeet;
  private double lastRightFeet;

  public DriveAtAngle(DriveTrainSubsystem driveTrainSubsystem, double ft, double speed, double angle) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.ft = ft;
    this.angle = angle;
    this.speed = speed;
    // addRequirements(driveTrainSubsystem);
    double[] tPid = driveTrainSubsystem.getTurnPid();
    if (Math.abs(Robot.gyro.getGyroAngleZ() - angle) > 70) {
      tPid[1] = 0;
    }
    // pid = new PID(tPid[0], tPid[1], tPid[2]);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new AccelerateDrive(driveTrainSubsystem, 0, speed, WheelSide.both).schedule();
    finished = false;
    lastLeftFeet = driveTrainSubsystem.getLeftDriveFeet();
    lastRightFeet = driveTrainSubsystem.getRightDriveFeet();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distence = (driveTrainSubsystem.getLeftDriveFeet()-lastLeftFeet + driveTrainSubsystem.getRightDriveFeet()-lastRightFeet)/2;
    double decelDistence = Robot.shuffleBoard.autoTuneAccelDistence.getDouble(0) *speed*1.1;
    if (distence + decelDistence >= ft) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new AccelerateDrive(driveTrainSubsystem, speed, 0, WheelSide.both).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
