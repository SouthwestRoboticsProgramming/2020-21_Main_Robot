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

public class ClimbCommand extends CommandBase {
  ClimbSubsystem m_climbSubsystem;
  private boolean hookPlaced;
  private boolean hookHightReached;
  // private 

  public ClimbCommand(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.m_climbSubsystem = climbSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbOutput = Robot.robotContainer.getClimbOutput();
    if (hookPlaced) {
      m_climbSubsystem.setWinch(Math.round(climbOutput));
    } else {
      m_climbSubsystem.setElevatorVelocity(m_climbSubsystem.getElevatorVelocityPercent(climbOutput));

      //TODO: what does '4' signify?

      if (m_climbSubsystem.getElevatorHeight() >= 4 && !hookHightReached) {
        hookHightReached = true;
      }
      if (m_climbSubsystem.elevatorLowerLimit() && hookHightReached) {
        hookPlaced = true;
        m_climbSubsystem.stopElevator();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.stopElevator();
    m_climbSubsystem.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}