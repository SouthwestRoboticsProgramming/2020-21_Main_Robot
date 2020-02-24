/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoSequence extends SequentialCommandGroup {
  /**
   * Creates a new AutoSequence.
   */
  public AutoSequence(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new PathBNew(driveTrainSubsystem, ballSubsystem),
      new PathC(driveTrainSubsystem, ballSubsystem)
      // new PathD(driveTrainSubsystem, ballSubsystem)
    );
  }
}
