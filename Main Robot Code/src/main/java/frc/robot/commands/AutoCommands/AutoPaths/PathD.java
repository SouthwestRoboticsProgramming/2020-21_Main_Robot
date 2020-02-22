/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoCommands.DriveDistence;
import frc.robot.commands.AutoCommands.FollowWithLimelight;
import frc.robot.commands.AutoCommands.TurnToAngle;
import frc.robot.commands.AutoCommands.setBallMode;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.subsystems.DriveTrainSubsystem.Wheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathD extends SequentialCommandGroup {
  public PathD(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    
    super(
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathCDistence2.getDouble(0), Robot.shuffleBoard.PathCSpeed2.getDouble(0)),
      new setBallMode(ballSubsystem, ballMode.intake),
      new FollowWithLimelight(driveTrainSubsystem, Robot.shuffleBoard.PathDLimelightTime2.getDouble(0), Robot.shuffleBoard.PathDLimelightSpeed2.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathDAngle3.getDouble(0), Wheel.both),
      new setBallMode(ballSubsystem, ballMode.hold),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathDDistence4.getDouble(0), Robot.shuffleBoard.PathDSpeed4.getDouble(0))
      );
    
  }
}
