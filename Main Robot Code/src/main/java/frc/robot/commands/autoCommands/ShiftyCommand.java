/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ShiftyCommand extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double frictionForce;
	private double maxSpeed;
  private double speed;
  private double currentSpeed;
	private boolean shiftyAdd = true;
  
  public ShiftyCommand(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShiftyCommand.initialize()");
    frictionForce = Robot.shuffleBoard.driveShiftyFrictionForce.getDouble(0);
    maxSpeed = Robot.shuffleBoard.driveShiftyMaxSpeed.getDouble(0);
    speed = Robot.shuffleBoard.driveShiftySpeed.getDouble(0);
    currentSpeed = 0;
    shiftyAdd = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double set;
    if (shiftyAdd) {
      set = currentSpeed + speed;
    } else {
      set = currentSpeed - speed;
    }
    System.out.println("ShiftyCommand.execute() set1 = " + set);
    if (-frictionForce < set && frictionForce > set) {
      if (set>0) {
        set = frictionForce;
      } else {
        set = -frictionForce;
      }
    }
    System.out.println("ShiftyCommand.execute() set2 = " + set);
    System.out.println("ShiftyCommand.execute() shiftyAdd = " + shiftyAdd);
    driveTrainSubsystem.driveMotors(set, set);
    if (Math.abs(set) >= maxSpeed) {
      shiftyAdd = !shiftyAdd;
    }
    currentSpeed = set;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ShiftyCommand.end()");
    driveTrainSubsystem.driveMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.robotContainer.getPOV() == -1;
  }
}
