package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveTrainSubsystem extends SubsystemBase {

  private WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave; //change to talon fx for main robot

  private int leftPort1 = 3,
              leftPort2 = 0,
              leftPort3 = -1,
              rightPort1 = 2,
              rightPort2 = 1,
              rightPort3 = -1;

  public DriveTrainSubsystem() {
    //setup motors
		leftMaster = new WPI_TalonSRX(leftPort1);
		leftSlave = new WPI_TalonSRX(leftPort2);
		rightMaster = new WPI_TalonSRX(rightPort1);
    	rightSlave = new WPI_TalonSRX(rightPort2);
    
    // RESET TALONS
		leftMaster.configFactoryDefault();
		leftSlave.configFactoryDefault();
		rightMaster.configFactoryDefault();
		rightSlave.configFactoryDefault();
		
		// LEFT MASTER
		leftMaster.setInverted(false);
		leftSlave.setInverted(false);
		leftMaster.setSensorPhase(false);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		
		// FOLLOW
		leftSlave.follow(leftMaster);
		
		// RIGHT MASTER
		rightMaster.setInverted(false);
		rightSlave.setInverted(false);
		rightMaster.setSensorPhase(false);	
		rightMaster.setNeutralMode(NeutralMode.Brake);

		// FOLLOW
		rightSlave.follow(rightMaster);
  }

  // GET INFO
  public double getLeftOutput() {
		return leftMaster.get();
	}

	public double getRightOutput() {
		return rightMaster.get();
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
		leftSlave.setNeutralMode(neutralMode);
		rightMaster.setNeutralMode(neutralMode);
		rightSlave.setNeutralMode(neutralMode);
  }
  
  public void driveMotors(double left, double right) {
      leftMaster.set(ControlMode.PercentOutput, left);
      rightMaster.set(ControlMode.PercentOutput, right);
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
