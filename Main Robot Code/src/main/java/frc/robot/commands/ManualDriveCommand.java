/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.CheesyDrive;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ManualDriveCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private CheesyDrive cheesyDrive = new CheesyDrive();
  private double[] tPid;
  private PID wallFollow;
  private PID limeLight;
  
  private double prevPowLeft = 0;
  private double prevPowRight = 0;
  private double prevRotation = 0;

  public static final double maxSpeedDiff = 0.08;

  public enum DriveType {
    arcade, cheezy, field, pid;
  }

  public ManualDriveCommand(DriveTrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);
    this.m_driveTrainSubsystem = driveTrainSubsystem;

    tPid = m_driveTrainSubsystem.getTurnPid();
    wallFollow = new PID(tPid[0], tPid[1], tPid[2]);
    limeLight = new PID(tPid[0], tPid[1], tPid[2]);
    wallFollow.setSetPoint(0);
    limeLight.setSetPoint(0);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TalonFXConfiguration drive = m_driveTrainSubsystem.getTFXC();
    //     drive.slot0.kF = Robot.shuffleBoard.driveFXPidF.getDouble(0);
    //     drive.slot0.kP = Robot.shuffleBoard.driveFXPidP.getDouble(0);
    //     drive.slot0.kI = Robot.shuffleBoard.driveFXPidI.getDouble(0);
    //     drive.slot0.kD = Robot.shuffleBoard.driveFXPidD.getDouble(0);
    //     m_driveTrainSubsystem.setTFXC(drive);
    //     System.out.print("updated control mode = " + Robot.shuffleBoard.driveType.getString(""));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print("ManualDriveExecute");
    double leftAuto = 0;
    double rightAuto = 0;

    // wallFollow
    double wallOffset = wallFollow.getOutput(Robot.gyro.getGyroAngleX());
    double wallEffectiveness = Robot.robotContainer.getWallEffeciveness();
    leftAuto += wallOffset * wallEffectiveness;
    rightAuto -= wallOffset * wallEffectiveness;

    // limelight
    double limelightOffset = limeLight.getOutput(Robot.limelight.limelightX());
    double limelightEffectiveness = Robot.robotContainer.getLimelightEffeciveness();
    leftAuto += limelightOffset * limelightEffectiveness;
    rightAuto -= limelightOffset * limelightEffectiveness;

    // driver
    double rotatMulti = .55;
    double x = Robot.robotContainer.getOneTurn();
    double y = Robot.robotContainer.getOneDrive();
    double driveSpeed = Robot.shuffleBoard.driveSpeed.getDouble(0);
    boolean quickTurn = Robot.robotContainer.getOneQuickTurn();

    if(getDriveType() == DriveType.arcade) { // arcade drive
      Robot.shuffleBoard.driveCurrentType.setString("arcadeDrive");
      double leftPow = limitAcceleration(y, prevPowLeft);
      double rightPow = limitAcceleration(y, prevPowRight);
      double rotation = limitAcceleration(x * rotatMulti, prevRotation);

      prevPowLeft = leftPow;
      prevPowRight = rightPow;
      prevRotation = rotation;
      m_driveTrainSubsystem.driveMotors((leftPow + rotation + leftAuto)*driveSpeed, (rightPow - rotation + rightAuto)*driveSpeed);
    } else if (getDriveType() == DriveType.cheezy) { // Cheesy Drive
      Robot.shuffleBoard.driveCurrentType.setString("cheezyDrive");
      var signal = cheesyDrive.cheesyDrive(y, x, quickTurn, false);
      double leftPow = signal.getLeft();
      double rightPow = signal.getRight();

      m_driveTrainSubsystem.driveMotors((leftPow + leftAuto)*driveSpeed, (rightPow + rightAuto)*driveSpeed);
    } else if (getDriveType() == DriveType.field) {
      Robot.shuffleBoard.driveCurrentType.setString("fieldDrive");
      double setAngle = getJoyAngle(x, y);
      setAngle = 1-wallEffectiveness;
      double output = getJoyDistence(x, y);
      output = limitAcceleration(output, (prevPowLeft + prevPowRight)/2);

      m_driveTrainSubsystem.driveAtAngle(output, setAngle, ControlMode.PercentOutput);
      prevPowLeft = output;
      prevPowRight = output;
    } else if (getDriveType() == DriveType.pid) {
      // double velocityConvert = m_driveTrainSubsystem.percentToVelocity(y);
      double velocityConvert = (double)y*20000;
      // m_driveTrainSubsystem.getLeftMaster().setVoltage(velocityConvert);
      // m_driveTrainSubsystem.getRightMaster().setVoltage(velocityConvert);
      m_driveTrainSubsystem.getLeftMaster().set(ControlMode.Position, velocityConvert);
      m_driveTrainSubsystem.getRightMaster().set(ControlMode.Position, velocityConvert);
      System.out.print("controller = " + y + " velocityConvert = " + velocityConvert);
    } else {
      Robot.shuffleBoard.driveCurrentType.setString("drivetypeNotFound");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSubsystem.stop();
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
    } else if (sbdt.equals("p")) {
      return DriveType.pid;
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
