/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.WheelSide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AccelDecelSequence extends SequentialCommandGroup {
  /**
   * Creates a new AccelDecelSequence.
   */
  public AccelDecelSequence(DriveTrainSubsystem driveTrainSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AccelerateDrive(driveTrainSubsystem, 0, 1, WheelSide.both),
    new AccelerateDrive(driveTrainSubsystem, 1, 0, WheelSide.both));
  }
}
