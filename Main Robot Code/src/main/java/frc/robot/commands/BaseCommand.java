/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BaseSubsystem;

/**
 * Have the robot drive tank style.
 */
public class BaseCommand extends CommandBase {
  private final BaseSubsystem m_baseSubsystem;

  /**
   * Creates a new TankDrive command.
   *
   * @param left       The control input for the left side of the drive
   * @param right      The control input for the right sight of the drive
   * @param drivetrain The drivetrain subsystem to drive
   */
  public BaseCommand(BaseSubsystem baseSubsystem) {
    m_baseSubsystem = baseSubsystem;
    addRequirements(m_baseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
  }
  
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

}