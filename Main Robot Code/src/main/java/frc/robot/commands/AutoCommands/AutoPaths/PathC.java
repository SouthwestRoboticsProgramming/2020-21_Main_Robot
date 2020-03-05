/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.autoCommands.DriveDistence;
import frc.robot.commands.autoCommands.ResetGyro;
import frc.robot.commands.autoCommands.TurnToAngle;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.WheelSide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathC extends SequentialCommandGroup {
  public PathC(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    
    super(
      new ResetGyro(),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathCAngle1.getDouble(0), WheelSide.right),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathCDistence2.getDouble(0), Robot.shuffleBoard.PathCSpeed2.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathCAngle3.getDouble(0), WheelSide.left)
      );
    
  }
}
