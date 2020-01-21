/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends CommandBase {
  ClimbSubsystem climbSubsystem;
  private boolean hookPlaced;
  private boolean hookHightReached;

  public Climb(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbOutput = Robot.robotContainer.climbOutput();
    if (hookPlaced) {
      climbSubsystem.setWinch(Math.round(climbOutput));
    } else {
      climbSubsystem.setelevator(climbSubsystem.getVelocity(climbOutput));
      if (climbSubsystem.getElevatorHight() >= 4 && !hookHightReached) {
        hookHightReached = true;
      }
      if (climbSubsystem.elevatorLowerLimit() && hookHightReached) {
        hookPlaced = true;
        climbSubsystem.stopElevator();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopElevator();
    climbSubsystem.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
