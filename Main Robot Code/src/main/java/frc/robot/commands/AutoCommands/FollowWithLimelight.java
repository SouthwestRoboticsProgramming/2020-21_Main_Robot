/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class FollowWithLimelight extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double angle;
  private PID pid;
  private boolean finished;
  private double speed;
  private double time;
  private Long startTime;

  public FollowWithLimelight(DriveTrainSubsystem driveTrainSubsystem, double time, double speed) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.speed = -speed;
    this.time = time*1000;
    // addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    angle = Robot.limelight.getX();
    double[] tPid = driveTrainSubsystem.getTurnPid();
    System.out.println("switch = " + Math.abs(angle - Robot.gyro.getGyroAngleZ()));
    pid = new PID(tPid[0], tPid[1], tPid[2]);
    pid.setSetPoint(angle);
    finished = false;
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = Robot.limelight.getX();
    pid.setSetPoint(angle);
    double output = -pid.getOutput(Robot.gyro.getGyroAngleZ()) * Robot.shuffleBoard.followWithLimelight.getDouble(0);
    if (output > 0) {
      output += Robot.shuffleBoard.driveTurnPidF.getDouble(0);
    } else {
      output -= Robot.shuffleBoard.driveTurnPidF.getDouble(0);
    }
    System.out.println("limelight.execute() out = " + pid.getError());
      driveTrainSubsystem.driveMotors(.5*output + speed, -.5*output + speed);

    
    if (System.currentTimeMillis() >= startTime + (long)time) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.driveMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
