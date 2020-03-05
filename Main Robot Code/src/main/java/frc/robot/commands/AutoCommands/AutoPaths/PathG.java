/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.autoCommands.AccelerateDrive;
import frc.robot.commands.autoCommands.DriveDistence;
import frc.robot.commands.autoCommands.DriveTime;
import frc.robot.commands.autoCommands.setBallMode;
import frc.robot.commands.autoCommands.TurnToAngle;
import frc.robot.commands.autoCommands.Wait;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.WheelSide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathG extends SequentialCommandGroup {
  public PathG(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    // place to Start
    super(
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathGAngle1.getDouble(0), WheelSide.left),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathGDistence2.getDouble(0), Robot.shuffleBoard.PathGSpeed2.getDouble(0)),
      // new AccelerateDrive(driveTrainSubsystem, 0, -.25, Wheel.both)
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathGAngle3.getDouble(0), WheelSide.left),
      new DriveTime(driveTrainSubsystem, Robot.shuffleBoard.PathGTime4.getDouble(0), Robot.shuffleBoard.PathGSpeed4.getDouble(0))
      );
    System.out.println("PathA.PathA()");
    
  }
}
