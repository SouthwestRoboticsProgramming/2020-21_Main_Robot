/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem.Wheel;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TurnToAngle extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private double angle;
  private Wheel wheel;
  private PID pid;
  private boolean finished;

  public TurnToAngle(DriveTrainSubsystem driveTrainSubsystem, double angle, Wheel wheel) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.angle = angle;
    this.wheel = wheel;
    double[] tPid = driveTrainSubsystem.getTurnPid();
    pid = new PID(tPid[0], tPid[1], tPid[2]);
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetPoint(angle);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.getOutput(Robot.gyro.getGyroAngleZ());
    if (wheel == Wheel.left) {
      driveTrainSubsystem.driveMotors(output, 0);
    } else {
      driveTrainSubsystem.driveMotors(0, -output);
    }

    if (Robot.gyro.getAccelInstantZ() < 1) {
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
