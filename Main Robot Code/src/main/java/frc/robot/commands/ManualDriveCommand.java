/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.CheesyDrive;
import frc.lib.Lib;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ManualDriveCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private CheesyDrive cheesyDrive = new CheesyDrive();
  private double[] tPid;
  private PID wallFollow;
  private PID limeLight;
  private DriveType driveType;
  
  //Previous l
  private double prevPowLeft = 0;
  private double prevPowRight = 0;
  private double prevRotation = 0;
  private double prevAngle = 0;

  //Maximum change in motor speed per execute
  public static final double maxSpeedDiff = 0.08;
  public static final double rotatMulti = .55;
  public static final double joystickDeadzone = .1;


  public enum DriveType {
    arcade, cheezy, field, pid;
  }

  public ManualDriveCommand(DriveTrainSubsystem driveTrainSubsystem, DriveType driveType) {
    addRequirements(driveTrainSubsystem);
    this.m_driveTrainSubsystem = driveTrainSubsystem;
    this.driveType = driveType;

    tPid = m_driveTrainSubsystem.getTurnPid();
    wallFollow = new PID(tPid[0], tPid[1], tPid[2]);
    limeLight = new PID(tPid[0], tPid[1], tPid[2]);
    wallFollow.setSetPoint(0);
    limeLight.setSetPoint(0);
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

    // wallFollow -- face aliance station
    double wallOffset = -wallFollow.getOutput(Robot.gyro.getGyroAngleZ());
    double wallEffectiveness = Robot.robotContainer.getWallEffeciveness();
    leftAuto += wallOffset * wallEffectiveness * Robot.shuffleBoard.driveStraightEffectiveness.getDouble(0);
    rightAuto -= wallOffset * wallEffectiveness *  Robot.shuffleBoard.driveStraightEffectiveness.getDouble(0);

    // limelight
    double limelightOffset = limeLight.getOutput(Robot.limelight.getX());
    double limelightEffectiveness = Robot.robotContainer.getLimelightEffeciveness();
    leftAuto += limelightOffset * limelightEffectiveness *  Robot.shuffleBoard.driveLimelightEffectiveness.getDouble(0);
    rightAuto -= limelightOffset * limelightEffectiveness *  Robot.shuffleBoard.driveLimelightEffectiveness.getDouble(0);

    // driver
    double xLeft = -Robot.robotContainer.getLeftTurn();
    double yLeft = -Robot.robotContainer.getLeftDrive();
    // xLeft = getDeadzone(xLeft, joystickDeadzone);
    // yLeft = getDeadzone(yLeft, joystickDeadzone);
    // xRight = getDeadzone(xRight, joystickDeadzone);
    // yRight = getDeadzone(yRight, joystickDeadzone);

    double driveSpeed = Robot.shuffleBoard.driveSpeed.getDouble(0);
    double arcadeSpeed = Robot.shuffleBoard.driveArcadeSpeed.getDouble(1);
    boolean quickTurn = Robot.robotContainer.getOneQuickTurn();

    if(getDriveType() == DriveType.arcade) { // arcade drive
      System.out.println("ManualDriveCommand.execute() arcade");
      Robot.shuffleBoard.driveCurrentType.setString("arcadeDrive");
      // double leftPow = limitAcceleration(yLeft, prevPowLeft);
      // double rightPow = limitAcceleration(yLeft, prevPowRight);
      // double rotation = limitAcceleration(xLeft * rotatMulti, prevRotation);

      double leftPow = yLeft;
      double rightPow = yLeft;
      double rotation = xLeft * Robot.shuffleBoard.driveArcadeTurn.getDouble(0);

      prevPowLeft = leftPow;
      prevPowRight = rightPow;
      prevRotation = rotation;
      m_driveTrainSubsystem.driveMotors((leftPow + rotation + leftAuto)*arcadeSpeed, (rightPow - rotation + rightAuto)*arcadeSpeed);
    } else if (getDriveType() == DriveType.cheezy) { // Cheesy Drive
      Robot.shuffleBoard.driveCurrentType.setString("cheezyDrive");
      var signal = cheesyDrive.cheesyDrive(yLeft, xLeft, quickTurn, false);
      double leftPow = signal.getLeft();
      double rightPow = signal.getRight();

      m_driveTrainSubsystem.driveMotors((leftPow + leftAuto)*driveSpeed, (rightPow + rightAuto)*driveSpeed);
    } else if (getDriveType() == DriveType.field) { //Field oriented driving
      Robot.shuffleBoard.driveCurrentType.setString("fieldDrive");
      double setAngle = getJoyAngle(xLeft, xLeft);

      System.out.println("joy angle = " + setAngle);
      SmartDashboard.putNumber("setAngle", setAngle);
      // setAngle = 1-wallEffectiveness;
      double output = 0; 
      // double output = driveSpeed*getJoyDistance(xRight, yRight);
      System.out.print("joyDistence = " + output);
      if (setAngle == 0) {
        setAngle = prevAngle;
      }
      
      m_driveTrainSubsystem.driveAtAngle(output, setAngle, ControlMode.PercentOutput);
      prevPowLeft = output;
      prevPowRight = output;
      if (setAngle != 0) {prevAngle = setAngle;}
    } else if (getDriveType() == DriveType.pid) {
      // double velocityConvert = m_driveTrainSubsystem.percentToVelocity(y);

      // m_driveTrainSubsystem.getLeftMaster().set(ControlMode.Velocity, velocityConvert);
      // m_driveTrainSubsystem.getRightMaster().set(ControlMode.Velocity, velocityConvert);
      // System.out.print("controller = " + y + " velocityConvert = " + velocityConvert);
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
    return driveType;
    // String sbdt = Robot.shuffleBoard.driveType.getString("");
    // if (sbdt.equals("a")) {
    //   return DriveType.arcade;
    // } else if (sbdt.equals("c")) {
    //   return DriveType.cheezy;
    // } else if (sbdt.equals("f")) {
    //   return DriveType.field;
    // } else if (sbdt.equals("p")) {
    //   return DriveType.pid;
    // } else {
    //   return DriveType.arcade;
    // }
  }

  private double getJoyAngle(double jX, double jY) {
    double angle = Math.toDegrees(Math.atan(jX / jY));
    if (jX > 0 && jY > 0) { // I
      // return (90+angle) + 90;
      return angle;
    } else if (jX < 0 && jY > 0) { // II
      // return -(90-angle) + 90;
        return angle;
    } else if (jX < 0 && jY < 0) { // III
      return -(90-angle) - 90;
      // return angle;
    } else if (jX > 0 && jY < 0) { // IV
      return (90+angle) + 90;
      // return angle;
    } else {
      return 0;
    }
  }

  private double getJoyDistance(double jX, double jY) {
    double d = Math.sqrt(Math.pow(jX, 2) + Math.pow(jY, 2));
    Lib lib = new Lib();
    d = lib.setRange(d, -1, 1);
    return d;
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
