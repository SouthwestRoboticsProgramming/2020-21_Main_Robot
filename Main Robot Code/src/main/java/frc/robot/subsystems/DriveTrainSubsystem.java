package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ADIS16448_IMU;
import frc.lib.Lib;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

	private final WPI_TalonFX leftMaster, leftSlave1, leftSlave2, 
							  rightMaster, rightSlave1, rightSlave2;
	private final double maxVelocity = 20000;
	private final double maxSpeed = 1;
	private Constants constants = new Constants();
	private Pose2d position;
	private DifferentialDriveOdometry differentialDriveOdometry;
	private TalonFXConfiguration fxConfig;

	

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

		// PID
		fxConfig = new TalonFXConfiguration();
		fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		fxConfig.neutralDeadband = .001;
		fxConfig.slot0.kF = Robot.shuffleBoard.driveFXPidF.getDouble(0);
		fxConfig.slot0.kP = Robot.shuffleBoard.driveFXPidP.getDouble(0);
		fxConfig.slot0.kI = Robot.shuffleBoard.driveFXPidI.getDouble(0);
		fxConfig.slot0.kD = Robot.shuffleBoard.driveFXPidD.getDouble(0);
		fxConfig.slot0.closedLoopPeakOutput = 1;
		fxConfig.openloopRamp = .5;


		rightMaster.configAllSettings(fxConfig);
		leftMaster.configAllSettings(fxConfig);
		
		// LEFT MASTER
		leftMaster.setInverted(true);
		leftSlave1.setInverted(true);
		leftSlave2.setInverted(true);
		leftMaster.setSensorPhase(true);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		leftMaster.setSelectedSensorPosition(0, 0, 30);
		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		
		// RIGHT MASTER
		rightMaster.config_kP(0, 0);
		rightMaster.config_kI(0, 0);
		rightMaster.config_kD(0, 0);
		rightMaster.config_kF(0, 0);
		rightMaster.configClosedloopRamp(.5);
		rightMaster.setInverted(false);
		rightSlave1.setInverted(false);
		rightSlave2.setInverted(false);
		rightMaster.setSensorPhase(true);
		rightMaster.setSelectedSensorPosition(0, 0, 30);	
		rightMaster.setNeutralMode(NeutralMode.Brake);
		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);
		
		position = new Pose2d(0,0,getRHeading());
		differentialDriveOdometry = new DifferentialDriveOdometry(getRHeading());
		differentialDriveOdometry.resetPosition(position, getRHeading());
	}

	public TalonFXConfiguration getTFXC() {
		return fxConfig;
	}

	public void setTFXC(TalonFXConfiguration config) {
		fxConfig = config;
		rightMaster.configAllSettings(fxConfig);
		leftMaster.configAllSettings(fxConfig);
	}

	// GET INFO
	public Rotation2d getRHeading() {
		return Rotation2d.fromDegrees(getHeading());
	}
	public double getHeading() {
		return 0;
		// return gyro.getGyroAngleX();
	}

	public WPI_TalonFX getLeftMaster() {
		return leftMaster;
	}

	public WPI_TalonFX getRightMaster() {
		return rightMaster;
	}

	public double getLeftOutput() {
		return leftMaster.get();
	}

	public double getRightOutput() {
		return rightMaster.get();
	}

	//GET ENCODOR OUTPUT
	public int getLeftDriveEncoderTicks() {
		return leftMaster.getSelectedSensorPosition();
	}

	public int getRightDriveEncoderTicks() {
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
		return (percent * maxVelocity)*2;
	}

	public void driveMotors(double left, double right) {
		driveMotors(left, right, ControlMode.PercentOutput);
		Robot.shuffleBoard.driveLeftOutput.setDouble(leftMaster.getMotorOutputPercent());
		Robot.shuffleBoard.driveRightOutput.setDouble(rightMaster.getMotorOutputPercent());
	}	

	public void stop() {
	leftMaster.stopMotor();
	rightMaster.stopMotor();
	}

	private double ticksToMeters(int ticks) {
		return(constants.wheelCircumferenceMeters / constants.encoderTicksPerRotation) * ticks;
	}

  @Override
  public void periodic() {
	  differentialDriveOdometry.update(getRHeading(), ticksToMeters(getLeftDriveEncoderTicks()) , ticksToMeters(getRightDriveEncoderTicks()));
	  Robot.shuffleBoard.drivePosition.setString(differentialDriveOdometry.getPoseMeters().toString());
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
	// System.out.println("angle = " + angle);
	PID pid = new PID(getTurnPid()[0], getTurnPid()[1], getTurnPid()[2]);
	pid.setSetPoint(angle);
	pid.setActual(Robot.getAngle());
	pid.setError(getError(Robot.getAngle(), angle));
	
	// System.out.println("angleOffset = " + pid.getOutput() + " error = " + pid.getError());
	Lib lib = new Lib();
	output = lib.setRange(output, -maxSpeed, maxSpeed);
	double leftOutput = -output - pid.getOutput();
	double rightOutput = -output + pid.getOutput();
	leftMaster.set(controlMode, leftOutput);
	rightMaster.set(controlMode, rightOutput);
			Robot.shuffleBoard.driveLeftOutput.setDouble(leftMaster.getMotorOutputPercent());
			Robot.shuffleBoard.driveRightOutput.setDouble(rightMaster.getMotorOutputPercent());
}

public static double getError(double act, double set) {
	if (act >= 0) {
		double a = -(-set+act);
		double b = 360-(act - set);
		if (a <= 180 && a >= -180) {
			return a;
		} else {
			return b;
		}
	} else {
		double c = -(-set+act);
		double d = -(360-(-act + set));
		if (c <= 180 && c >= -180) {
			return c;
		} else {
			return d;
		}
	}		
}

}
