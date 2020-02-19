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

public class GetAcclDecelDistence extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double lastLeftFeet;
  private double lastRightFeet;

  public GetAcclDecelDistence(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("GetAcclDecelDistence.initialize()");
    lastLeftFeet = driveTrainSubsystem.getLeftDriveFeet();
    lastRightFeet = driveTrainSubsystem.getRightDriveFeet();
    new AccelDecelSequence(driveTrainSubsystem).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double distence = (driveTrainSubsystem.getLeftDriveFeet()-lastLeftFeet + driveTrainSubsystem.getRightDriveFeet()-lastRightFeet)/2;
    Robot.shuffleBoard.autoTuneAccelDistence.setDouble(distence);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
