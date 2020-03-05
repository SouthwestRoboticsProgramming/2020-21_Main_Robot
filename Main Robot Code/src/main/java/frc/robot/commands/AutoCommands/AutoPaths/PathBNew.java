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
import frc.robot.commands.AutoCommands.TurnToAngle;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.WheelSide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathBNew extends SequentialCommandGroup {
  public PathBNew(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    // start to placement
    super(
      new Wait(Robot.shuffleBoard.PathBnWait0.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBnAngle1.getDouble(0), WheelSide.left),
      new Wait(.5),
      new DriveDistence(driveTrainSubsystem, Robot.shuffleBoard.PathBnDistence2.getDouble(0), Robot.shuffleBoard.PathBnSpeed2.getDouble(0)),
      new TurnToAngle(driveTrainSubsystem, Robot.shuffleBoard.PathBnAngle3.getDouble(0), WheelSide.left),
      new DriveTime(driveTrainSubsystem, Robot.shuffleBoard.PathBnTime4.getDouble(0), Robot.shuffleBoard.PathBnSpeed4.getDouble(0)),
      new setBallMode(ballSubsystem, ballMode.unloadOutput),
      new Wait(Robot.shuffleBoard.PathBnWait5.getDouble(0)),
      new setBallMode(ballSubsystem, ballMode.hold)
      );
    System.out.println("PathA.PathA()");
    
  }
}
