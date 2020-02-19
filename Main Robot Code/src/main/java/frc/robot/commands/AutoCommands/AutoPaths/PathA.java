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
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.setBallMode;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathA extends SequentialCommandGroup {
  public PathA(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    
    super(
      new Wait(Robot.shuffleBoard.PathAWait0.getDouble(0)),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathADistence1.getDouble(0), Robot.shuffleBoard.PathASpeed1.getDouble(0)),
      new DriveTime(driveTrainSubsystem, Robot.shuffleBoard.PathATime2.getDouble(0), Robot.shuffleBoard.PathASpeed2.getDouble(0)),
      new setBallMode(ballSubsystem, ballMode.unloadOutput),
      new Wait(Robot.shuffleBoard.PathAWait3.getDouble(0)),
      new setBallMode(ballSubsystem, ballMode.hold)
      );
    
  }
}
