/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.Wheel;

public class DriveDistance extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double ft;
  private Wheel wheel;
  private double speed;
  private boolean finished;
  private double lastLeftFeet;
  private double lastRightFeet;

  public DriveDistance(DriveTrainSubsystem driveTrainSubsystem, double ft, double speed, Wheel wheel) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.ft = ft;
    this.wheel = wheel;
    this.speed = speed;
    addRequirements(driveTrainSubsystem);
  }

  public DriveDistance(DriveTrainSubsystem driveTrainSubsystem, double ft, double speed) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.ft = ft;
    this.wheel = Wheel.both;
    this.speed = speed;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new AccelerateDrive(driveTrainSubsystem, 0, speed, wheel).schedule();
    finished = false;
    lastLeftFeet = driveTrainSubsystem.getLeftDriveFeet();
    lastRightFeet = driveTrainSubsystem.getRightDriveFeet();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Distance = (driveTrainSubsystem.getLeftDriveFeet()-lastLeftFeet + driveTrainSubsystem.getRightDriveFeet()-lastRightFeet)/2;
    System.out.println("Distance = " + Distance);
    double decelDistance = Robot.shuffleBoard.autoTuneAccelDistance.getDouble(0) *speed*1.1;
    System.out.println("decelDistance = " + decelDistance);
    if (ft > 0) {
      if (Distance + decelDistance >= ft) {
        finished = true;
      }
    } else {
      if (Distance + decelDistance <= ft) {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new AccelerateDrive(driveTrainSubsystem, speed, 0, wheel).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
