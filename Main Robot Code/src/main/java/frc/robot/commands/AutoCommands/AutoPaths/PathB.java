/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem.Wheel;
import frc.robot.commands.AutoCommands.DriveDistence;
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.TurnToAngle;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathB extends SequentialCommandGroup {
  public PathB(DriveTrainSubsystem driveTrainSubsystem) {
    super(
      new Wait(Robot.shuffleBoard.PathBWait0.getDouble(0)),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathBDistence1.getDouble(0), Robot.shuffleBoard.PathBSpeed1.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBAngle2.getDouble(0), Wheel.left),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathBDistence3.getDouble(0), Robot.shuffleBoard.PathBSpeed3.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBAngle4.getDouble(0), Wheel.right),
      new DriveTime(driveTrainSubsystem, Robot.shuffleBoard.PathBTime5.getDouble(0), Robot.shuffleBoard.PathBSpeed5.getDouble(0), Wheel.both)
    );
  }
}
