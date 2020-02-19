/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
  private double seconds;
  private Long endTime;
  private boolean finished;

  public Wait(double seconds) {
    this.seconds = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTime = System.currentTimeMillis() + (long)(seconds * 1000);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis() > endTime) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
