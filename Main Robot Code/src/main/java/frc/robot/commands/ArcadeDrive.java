/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.CheesyDrive;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {

  private CheesyDrive cheesyDrive = new CheesyDrive();
  private DriveTrain driveTrain;

  private double prevPowLeft = 0;
  private double prevPowRight = 0;
  private double prevRotation = 0;

  public static final double maxSpeedDiff = 0.08;

  public ArcadeDrive(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPow = Robot.robotContainer.oneDrive();
    double rightPow = leftPow;
    double rotation = Robot.robotContainer.oneTurn() * .55;

    if (!Robot.shuffleBoard.cheezyDrive.getBoolean(true)) { // Arcade Drive
      leftPow = limitAcceleration(leftPow, prevPowLeft);
      rightPow = limitAcceleration(rightPow, prevPowRight);
      rotation = limitAcceleration(rotation, prevRotation);

      prevPowLeft = leftPow;
      prevPowRight = rightPow;
      prevRotation = rotation;
    } else { // Cheesy Drive
      boolean quickTurn = Robot.robotContainer.oneQuickTurn();
      var signal = cheesyDrive.cheesyDrive(leftPow, rotation, quickTurn, false);
      leftPow = signal.getLeft();
      rightPow = signal.getRight();

      // CheezyDrive takes care of rotation so set to 0 to keep or code from adjusting
      rotation = 0;
    }
    // Robot.driveTrain.driveMotors(leftPow + rotation, rightPow - rotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double limitAcceleration(double setPow, double prevPow) {
    if (Math.abs(setPow - prevPow) > maxSpeedDiff) {
        if (setPow > prevPow) {
            return prevPow + maxSpeedDiff;
        } else if (setPow < prevPow) {
            return prevPow - maxSpeedDiff;
        } else {
            return prevPow;
        }
    } else {
        return prevPow;
    }
  }
}
