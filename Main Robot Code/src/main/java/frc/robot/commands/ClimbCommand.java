/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
  private ClimbSubsystem m_climbSubsystem;

  
  // private 

  public ClimbCommand(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.m_climbSubsystem = climbSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbOutput = getDeadzone(Robot.robotContainer.getClimbOutput(), Robot.shuffleBoard.climbDeadZone.getDouble(0));
    boolean winchControlled = false;
    // climbOutput = climbOutput * Robot.shuffleBoard.climbElevatorSpeed.getDouble(0);
    m_climbSubsystem.setElevator(climbOutput);
    
    if (climbOutput * Robot.robotContainer.getBothClimb() < 0) {
      winchControlled = true;
      System.out.println("bothClimb = " + Robot.robotContainer.getBothClimb());
      m_climbSubsystem.setWinch(-.5 * climbOutput * Robot.robotContainer.getBothClimb());
    }
    if (!winchControlled) {
      if (Robot.robotContainer.getWinchOutput()) {
        double winchSpeed = Robot.shuffleBoard.climbWinchSpeed.getDouble(0);
        m_climbSubsystem.setWinch(winchSpeed);
      } else {
        m_climbSubsystem.setWinch(0);
      }
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setElevator(0);
    m_climbSubsystem.setWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static double getDeadzone(double act, double deadZone) {
		if (Math.abs(act) < deadZone) {
			return 0;
		} else {
			if (act > 0) {
				return (act-deadZone) * (1/(1-deadZone));
			} else {
				return (act+deadZone) * (1/(1-deadZone));
			}
		}
	}
}