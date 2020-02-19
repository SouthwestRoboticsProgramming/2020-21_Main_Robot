/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
<<<<<<< HEAD
import frc.robot.commands.autoCommands.DriveDistance;
import frc.robot.commands.autoCommands.DriveTime;
import frc.robot.commands.autoCommands.TurnToAngle;
import frc.robot.commands.autoCommands.Wait;
import frc.robot.commands.autoCommands.setBallMode;
import frc.robot.subsystems.BallSubsystem;
=======
import frc.robot.subsystems.DriveTrainSubsystem.Wheel;
import frc.robot.commands.AutoCommands.DriveDistence;
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.TurnToAngle;
import frc.robot.commands.AutoCommands.Wait;
>>>>>>> parent of b1da761... added  auto
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathB extends SequentialCommandGroup {
  public PathB(DriveTrainSubsystem driveTrainSubsystem) {
    super(
      new Wait(Robot.shuffleBoard.PathBWait0.getDouble(0)),
<<<<<<< HEAD
      new DriveDistance(driveTrainSubsystem, Robot.shuffleBoard.PathBDistance1.getDouble(0), Robot.shuffleBoard.PathBSpeed1.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBAngle2.getDouble(0), Wheel.right),
      new DriveDistance(driveTrainSubsystem, Robot.shuffleBoard.PathBDistance3.getDouble(0), Robot.shuffleBoard.PathBSpeed3.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBAngle4.getDouble(0), Wheel.left),
      new DriveTime(driveTrainSubsystem, Robot.shuffleBoard.PathBTime5.getDouble(0), Robot.shuffleBoard.PathBSpeed5.getDouble(0), Wheel.both),
      new setBallMode(ballSubsystem, ballMode.unloadOutput),
      new Wait(Robot.shuffleBoard.PathBWait6.getDouble(0)),
      new setBallMode(ballSubsystem, ballMode.hold)
=======
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathBDistence1.getDouble(0), Robot.shuffleBoard.PathBSpeed1.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBAngle2.getDouble(0), Wheel.left),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathBDistence3.getDouble(0), Robot.shuffleBoard.PathBSpeed3.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBAngle4.getDouble(0), Wheel.right),
      new DriveTime(driveTrainSubsystem, Robot.shuffleBoard.PathBTime5.getDouble(0), Robot.shuffleBoard.PathBSpeed5.getDouble(0), Wheel.both)
>>>>>>> parent of b1da761... added  auto
    );
  }
}
