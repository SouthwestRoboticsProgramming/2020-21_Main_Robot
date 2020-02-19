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
import frc.robot.commands.AutoCommands.TurnToAngle;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.Wheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathF extends SequentialCommandGroup {
  public PathF(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    
    super(
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathFAngle1.getDouble(0), Wheel.right),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathFDistence2.getDouble(0), Robot.shuffleBoard.PathFSpeed2.getDouble(0))
      );
    
  }
}
