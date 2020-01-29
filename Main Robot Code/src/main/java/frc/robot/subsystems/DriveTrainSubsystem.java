package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Lib;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

	private final WPI_TalonFX leftMaster, leftSlave1, leftSlave2, rightMaster, rightSlave1, rightSlave2; //change to talon fx for main robot
	private final double maxVelocity = 100;
	private final double maxSpeed = 1;

	

	public DriveTrainSubsystem() {
	//setup motors
		leftMaster = new WPI_TalonFX(Constants.leftPort1);
		leftSlave1 = new WPI_TalonFX(Constants.leftPort2);
		leftSlave2 = new WPI_TalonFX(Constants.leftPort3);
		rightMaster = new WPI_TalonFX(Constants.rightPort1);
		rightSlave1 = new WPI_TalonFX(Constants.rightPort2);
		rightSlave2 = new WPI_TalonFX(Constants.rightPort3);

	// RESET TALONS
		leftMaster.configFactoryDefault();
		leftSlave1.configFactoryDefault();
		leftSlave2.configFactoryDefault();
		rightMaster.configFactoryDefault();
		rightSlave1.configFactoryDefault();
		rightSlave2.configFactoryDefault();
		
		// LEFT MASTER
		leftMaster.setInverted(true);
		leftSlave1.setInverted(true);
		leftSlave2.setInverted(true);
		leftMaster.setSensorPhase(false);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		
		// FOLLOW
		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		
		// RIGHT MASTER
		rightMaster.setInverted(false);
		rightSlave1.setInverted(false);
		rightSlave2.setInverted(false);
		rightMaster.setSensorPhase(false);	
		rightMaster.setNeutralMode(NeutralMode.Brake);

		// FOLLOW
		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);
	}

	// GET INFO
	public double getLeftOutput() {
		return leftMaster.get();
	}

	public double getRightOutput() {
		return rightMaster.get();
	}

	public double[] getDrivePid() {
		double[] drivePID = new double[] {Robot.shuffleBoard.drivePidP.getDouble(0),
										  Robot.shuffleBoard.drivePidI.getDouble(0),
										  Robot.shuffleBoard.drivePidD.getDouble(0)};
		return drivePID;
	}

	public double[] getTurnPid() {
		double[] turnPID = new double[] {Robot.shuffleBoard.driveTurnPidP.getDouble(0),
										 Robot.shuffleBoard.driveTurnPidI.getDouble(0),
										 Robot.shuffleBoard.driveTurnPidD.getDouble(0)};
		return turnPID;
	}

	//GET ENCODOR OUTPUT
	public double getLeftDriveEncoder() {
		return leftMaster.getSelectedSensorPosition();
	}

	public double getRightDriveEncoder() {
		return rightMaster.getSelectedSensorPosition();
	}

	public double getLeftDriveFeet() {
		return leftMaster.getSelectedSensorPosition() / 1024 * Math.PI * 6;
	}

	public double getRightDriveFeet() {
		return rightMaster.getSelectedSensorPosition() / 1024 * Math.PI * 6;
	}

	// SET BRAKE MODE OF MOTORS
	public void setBrakeMode(boolean mode) {
		NeutralMode neutralMode;
		if (mode) {
			neutralMode = NeutralMode.Brake;
		} else {
			neutralMode = NeutralMode.Coast;
		}
		leftMaster.setNeutralMode(neutralMode);
		leftSlave1.setNeutralMode(neutralMode);
		leftSlave2.setNeutralMode(neutralMode);
		rightMaster.setNeutralMode(neutralMode);
		rightSlave1.setNeutralMode(neutralMode);
		rightSlave2.setNeutralMode(neutralMode);
	}

	public double getLeftVelocity() {
		return leftMaster.getSelectedSensorVelocity(0);
	}

	public double getLeftVelocityPercent() {
		return leftMaster.getSelectedSensorVelocity(0) / maxVelocity;
	}

	public double getRightVelocity() {
		return rightMaster.getSelectedSensorVelocity(0);
	}

	public double getRightVelocityPercent() {
		return rightMaster.getSelectedSensorVelocity(0) / maxVelocity;
	}

	public double percentToVelocity(double percent) {
		return percent * maxVelocity;
	}

	public void driveMotors(double left, double right) {
		driveMotors(left, right, ControlMode.PercentOutput);
		Robot.shuffleBoard.driveLeftOutput.setDouble(leftMaster.getMotorOutputPercent());
		Robot.shuffleBoard.driveRightOutput.setDouble(rightMaster.getMotorOutputPercent());
	}	

	public void driveMotors(double left, double right, ControlMode controlMode) {
		Lib lib = new Lib();
		left = lib.setRange(left, -maxSpeed, maxSpeed);
		right = lib.setRange(right, -maxSpeed, maxSpeed);
		leftMaster.set(controlMode, left);
		rightMaster.set(controlMode, right);
				Robot.shuffleBoard.driveLeftOutput.setDouble(leftMaster.getMotorOutputPercent());
				Robot.shuffleBoard.driveRightOutput.setDouble(rightMaster.getMotorOutputPercent());
	}
	
	public void driveAtAngle(double output, double angle, ControlMode controlMode) {
		PID pid = new PID(getTurnPid()[0], getTurnPid()[1], getTurnPid()[2]);
		pid.setSetPoint(angle);
		pid.setActual(Robot.gyro.getGyroAngleX());
		Lib lib = new Lib();
		output = lib.setRange(output, -maxSpeed, maxSpeed);
		leftMaster.set(controlMode, output);
		rightMaster.set(controlMode, output);
				Robot.shuffleBoard.driveLeftOutput.setDouble(leftMaster.getMotorOutputPercent());
				Robot.shuffleBoard.driveRightOutput.setDouble(rightMaster.getMotorOutputPercent());
	}

	public void stop() {
	leftMaster.stopMotor();
	rightMaster.stopMotor();
	}

  @Override
  public void periodic() {
  }
}
