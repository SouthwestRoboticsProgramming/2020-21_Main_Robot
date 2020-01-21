/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.CheesyDrive;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ManualDrive extends CommandBase {

  private CheesyDrive cheesyDrive = new CheesyDrive();
  private DriveTrainSubsystem driveTrain;
  private double[] tPid = driveTrain.getTurnPid();
  private PID wallFollow = new PID(tPid[0], tPid[1], tPid[2]);
  private PID limeLight = new PID(tPid[0], tPid[1], tPid[2]);

  private double prevPowLeft = 0;
  private double prevPowRight = 0;
  private double prevRotation = 0;

  public static final double maxSpeedDiff = 0.08;

  public enum DriveType {
    arcade, cheezy, field;
  }

  public ManualDrive(DriveTrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
    wallFollow.setSetPoint(0);
    limeLight.setSetPoint(0);
    addRequirements(this.driveTrain);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftAuto = 0;
    double rightAuto = 0;

    // wallFollow
    double wallOffset = wallFollow.getOutput(Robot.gyro.getGyroAngleX());
    double wallEffectiveness = Robot.robotContainer.wallEffeciveness();
    leftAuto += wallOffset * wallEffectiveness;
    rightAuto -= wallOffset * wallEffectiveness;

    // limelight
    double limelightOffset = limeLight.getOutput(Robot.limelight.limelightX());
    double limelightEffectiveness = Robot.robotContainer.limelightEffeciveness();
    leftAuto += limelightOffset * limelightEffectiveness;
    rightAuto -= limelightOffset * limelightEffectiveness;

    // driver
    double rotatMulti = .55;
    double x = Robot.robotContainer.oneTurn();
    double y = Robot.robotContainer.oneDrive();
    boolean quickTurn = Robot.robotContainer.oneQuickTurn();

    if(getDriveType() == DriveType.arcade) { // arcade drive
      double leftPow = limitAcceleration(y, prevPowLeft);
      double rightPow = limitAcceleration(y, prevPowRight);
      double rotation = limitAcceleration(x * rotatMulti, prevRotation);

      prevPowLeft = leftPow;
      prevPowRight = rightPow;
      prevRotation = rotation;
      driveTrain.driveMotors(leftPow + rotation + leftAuto, rightPow - rotation + rightAuto);
    } else if (getDriveType() == DriveType.cheezy) { // Cheesy Drive
      var signal = cheesyDrive.cheesyDrive(y, x, quickTurn, false);
      double leftPow = signal.getLeft();
      double rightPow = signal.getRight();

      driveTrain.driveMotors(leftPow + leftAuto, rightPow + rightAuto);
    } else if (getDriveType() == DriveType.field) {
      double setAngle = getJoyAngle(x, y);
      setAngle = 1-wallEffectiveness;
      double output = getJoyDistence(x, y);

      driveTrain.driveAtAngle(output, setAngle, ControlMode.PercentOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
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

  private DriveType getDriveType() {
    String sbdt = Robot.shuffleBoard.driveType.getString("");
    if (sbdt.equals("a")) {
      return DriveType.arcade;
    } else if (sbdt.equals("c")) {
      return DriveType.cheezy;
    } else if (sbdt.equals("f")) {
      return DriveType.field;
    } else {
      return DriveType.arcade;
    }
  }

  private double getJoyAngle(double jX, double jY) {
    double angle = Math.toDegrees(Math.atan(jX / jY));
    if (jX > 0 && jY > 0) { // I
      return angle;
    } else if (jX < 0 && jY > 0) { // II
        return angle;
    } else if (jX < 0 && jY < 0) { // III
      return -(90-angle) - 90;
    } else if (jX > 0 && jY < 0) { // IV
      return (90+angle) + 90;
    } else {
      return 0;
    }
  }

  private double getJoyDistence(double jX, double jY) {
    return Math.sqrt(Math.pow(jX, 2) + Math.pow(jY, 2));
  }

}
