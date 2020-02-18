/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.autoCommands.DriveDistance;
import frc.robot.commands.autoCommands.DriveTime;
import frc.robot.commands.autoCommands.setBallMode;
import frc.robot.commands.autoCommands.TurnToAngle;
import frc.robot.commands.autoCommands.Wait;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.Wheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathBNew extends SequentialCommandGroup {
  public PathBNew(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    // start to placement
    super(
      new Wait(Robot.shuffleBoard.PathBnWait0.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBnAngle1.getDouble(0), Wheel.left),
      new Wait(1),
      new DriveDistance(driveTrainSubsystem, Robot.shuffleBoard.PathBnDistance2.getDouble(0), Robot.shuffleBoard.PathBnSpeed2.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBnAngle3.getDouble(0), Wheel.left),
      new DriveTime(driveTrainSubsystem, Robot.shuffleBoard.PathBnTime4.getDouble(0), Robot.shuffleBoard.PathBnSpeed4.getDouble(0)),
      new setBallMode(ballSubsystem, ballMode.unloadOutput),
      new Wait(Robot.shuffleBoard.PathBnWait5.getDouble(0)),
      new setBallMode(ballSubsystem, ballMode.hold)
      );
    
  }
}
