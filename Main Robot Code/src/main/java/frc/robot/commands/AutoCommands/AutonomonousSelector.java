/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD
import frc.robot.commands.autoCommands.autoPaths.AutoSequence;
import frc.robot.subsystems.BallSubsystem;
=======
import frc.robot.Robot;
import frc.robot.commands.AutoCommands.AutoPaths.PathA;
import frc.robot.commands.AutoCommands.AutoPaths.PathB;
>>>>>>> parent of b1da761... added  auto
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutonomonousSelector extends CommandBase {
  DriveTrainSubsystem driveTrainSubsystem;

  public AutonomonousSelector(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    System.out.println("AutonomonousSelector.initialize()");
    new AutoSequence(driveTrainSubsystem, ballSubsystem).schedule();
=======
    runPath(Robot.shuffleBoard.path1.getString(""));
    runPath(Robot.shuffleBoard.path2.getString(""));
    runPath(Robot.shuffleBoard.path3.getString(""));
    runPath(Robot.shuffleBoard.path4.getString(""));
>>>>>>> parent of b1da761... added  auto
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

  private void runPath(String path) {
    if (path.toUpperCase() == "A") {
      new PathA(driveTrainSubsystem);
    } else if (path.toUpperCase() == "B") {
      new PathB(driveTrainSubsystem);
    } else {

    }
  }
}
