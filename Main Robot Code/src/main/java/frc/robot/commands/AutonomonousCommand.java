/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomonousCommand extends CommandBase {
  /**
   * Creates a new AutonomonousCommand.
   */
  public AutonomonousCommand(double x, double y) {
    R<Doub> relPos = Robot.encoders.pos().sub(new R<Doub>(x,y));

    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.driveTrainSubsystem.driveAtAngle(Math.min(relPos.mag().div(5d),0.4) , Math.atan2(relPos.get(1),relPos.get(0)), DriveMode.cheezy);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
