/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem.Wheel;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTime extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double seconds;
  private Wheel wheel;
  private double speed;
  private long endTime;
  private boolean finished;

  public DriveTime(DriveTrainSubsystem driveTrainSubsystem, double seconds, double speed, Wheel wheel) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.seconds = seconds;
    this.wheel = wheel;
    this.speed = speed;
    addRequirements(driveTrainSubsystem);
  }

  public DriveTime(DriveTrainSubsystem driveTrainSubsystem, double seconds, double speed) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.seconds = seconds;
    this.wheel = Wheel.both;
    this.speed = speed;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveTime.initialize()");
    endTime = System.currentTimeMillis() + (long)(seconds * 1000) - (long)(Robot.shuffleBoard.autoTuneAcceleration.getDouble(0) * speed*1000);
    new AccelerateDrive(driveTrainSubsystem, 0, speed, wheel).schedule();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (System.currentTimeMillis() >= endTime) {
      finished = true;
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
