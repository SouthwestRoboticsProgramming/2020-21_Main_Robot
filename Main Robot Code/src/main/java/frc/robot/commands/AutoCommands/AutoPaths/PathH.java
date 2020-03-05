/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoCommands.AccelerateDrive;
import frc.robot.commands.AutoCommands.DriveDistence;
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.ResetGyro;
import frc.robot.commands.AutoCommands.setBallMode;
import frc.robot.commands.AutoCommands.TurnToAngle;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.WheelSide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathH extends SequentialCommandGroup {
  public PathH(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    // place to Start
    super(
      new Wait(.5),
      new ResetGyro(),
      new Wait(.5),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathHAngle1.getDouble(0), WheelSide.left),
      new DriveTime(driveTrainSubsystem, Robot.shuffleBoard.PathHTime2.getDouble(0), Robot.shuffleBoard.PathHSpeed2.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathHAngle3.getDouble(0), WheelSide.left)
      );
    
  }
}
