/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.WheelSubsystem;

public class WheelOfFortune extends CommandBase {
  private WheelSubsystem wheelSubsystem;
  private Spin spin;
  private WheelSubsystem.Color color;

  enum Spin {
    Revolutions,
    Position;
  }

  public WheelOfFortune(WheelSubsystem wheelSubsystem, DriveTrainSubsystem driveTrainSubsystem, Spin spin) {
    this.wheelSubsystem = wheelSubsystem;
    this.spin = spin;
    addRequirements(this.wheelSubsystem);
  }

  public WheelOfFortune(WheelSubsystem wheelSubsystem, DriveTrainSubsystem driveTrainSubsystem, Spin spin, WheelSubsystem.Color color) {
    this.wheelSubsystem = wheelSubsystem;
    this.spin = spin;
    this.color = color;
    addRequirements(this.wheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    


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
