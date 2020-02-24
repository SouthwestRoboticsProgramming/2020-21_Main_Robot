/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DriveWithLimelightToHp.WaitBeforeDrivingToHP;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
/* 
* Controls the ball subsystem.
* Input: Ball subsytem and the mode of the subsystem
* Output: Ball subsystem is set to th
*/
public class BallCommand extends CommandBase {
  private BallSubsystem m_ballSubsystem;
  private DriveTrainSubsystem driveTrainSubsystem;
  private ballMode mode;
  

  public BallCommand(BallSubsystem ballSubsystem, DriveTrainSubsystem driveTrainSubsystem, ballMode ballMode) {
    addRequirements(ballSubsystem);
    this.m_ballSubsystem = ballSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.mode = ballMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // new WaitBeforeDrivingToHP(driveTrainSubsystem).schedule();
    m_ballSubsystem.setBallMode(mode);
    if (mode == ballMode.unloadOutput || mode == ballMode.unloadIntake) {
      m_ballSubsystem.setBallCount(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
