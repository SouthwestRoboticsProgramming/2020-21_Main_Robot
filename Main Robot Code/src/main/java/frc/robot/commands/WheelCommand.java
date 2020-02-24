/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Lib;
import frc.lib.Looper.Loop;
import frc.lib.Looper.Looper;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.WheelSubsystem;

public class WheelCommand extends CommandBase {
  private final WheelSubsystem m_wheelSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private Looper powerRamper;
  private Spin spin;
  private WheelSubsystem.Color color;
  private long startTime;
  private boolean finished = false;
  private boolean phaseA = false;

  public enum Spin {
    Revolutions,
    Position;
  }
  
  public WheelCommand(WheelSubsystem wheelSubsystem, DriveTrainSubsystem driveTrainSubsystem, Spin spin) {
    System.out.println("WheelCommand.WheelCommand()");
    addRequirements(wheelSubsystem);
    this.m_wheelSubsystem = wheelSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.spin = spin;
  }

  public WheelCommand(WheelSubsystem wheelSubsystem, DriveTrainSubsystem driveTrainSubsystem,  Spin spin, WheelSubsystem.Color color) {
    this.m_wheelSubsystem = wheelSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.spin = spin;
    this.color = color;
    addRequirements(this.m_wheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WheelCommand.initialize()");
    startTime = System.currentTimeMillis();
    m_wheelSubsystem.setPushedState(true);
    m_wheelSubsystem.setSpinnerTalon(0);
    finished = false; 
    phaseA = false;
    try {
      Thread.sleep(750);
    } catch (Exception e) {
      //TODO: handle exception
    }
    while(!m_wheelSubsystem.getLimit()) {
      driveTrainSubsystem.driveMotors(.125, .125);
    }
    driveTrainSubsystem.driveMotors(-.125, -.125);
    try {
      Thread.sleep((long)Robot.shuffleBoard.wheelBackupTimems.getDouble(0));
    } catch (Exception e) {
      //TODO: handle exception
    }
    driveTrainSubsystem.driveMotors(0, 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double manualSpeed = getDeadzone(Robot.robotContainer.getWheelManual(), Robot.shuffleBoard.wheelManualDeadZone.getDouble(.5));
    if (Math.abs(manualSpeed) > 0) {
      phaseA = false;
    }
    if (!phaseA) {
      if (spin == Spin.Revolutions) {
        setSpinnerSpeed(1);
        double spinTime = Robot.shuffleBoard.wheelRevolutionsMS.getDouble(0);
        if (System.currentTimeMillis() - spinTime >= startTime || Robot.robotContainer.voidTask()) {finished = true;}
      } else if (spin == Spin.Position) {
        setSpinnerSpeed(.2);
        System.out.println("set color = " + getColor(color) + " color = " + m_wheelSubsystem.getColor());
        if (getColor(color) == m_wheelSubsystem.getColor() || Robot.robotContainer.voidTask()) {finished = true;}
      }
    } else {
      setSpinnerSpeed(manualSpeed);
      if (Robot.robotContainer.voidTask()) {
        finished = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WheelCommand.end()");
    m_wheelSubsystem.setSpinnerTalon(0);
    try {
     Thread.sleep(1000); 
    } catch (Exception e) {
      //TODO: handle exception
    }
    m_wheelSubsystem.setPushedState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  private WheelSubsystem.Color getColor(WheelSubsystem.Color color) {
    if (color == WheelSubsystem.Color.blue) {
        return WheelSubsystem.Color.red;
    } else if (color == WheelSubsystem.Color.green) {
      return WheelSubsystem.Color.yellow;
    } else if (color == WheelSubsystem.Color.red) {
      return WheelSubsystem.Color.blue;
    } else if (color == WheelSubsystem.Color.yellow) {
      return WheelSubsystem.Color.green;
    } else {
      // DriverStation.reportError("Color " + color + " not found!", true);
      return null;

    }
  }
  
  private void setSpinnerSpeed(double speed) {
    double add = Robot.shuffleBoard.wheelAcceleration.getDouble(0);
    double start = m_wheelSubsystem.getSpinnerTalon();
    while (Math.abs(start - speed) > add*2) {
      System.out.println("spinner Talon = " + m_wheelSubsystem.getSpinnerTalon());
      System.out.println("add = " + add);
      System.out.println("start = " + start);
      if (m_wheelSubsystem.getSpinnerTalon() < speed) {
        m_wheelSubsystem.setSpinnerTalon(start + add);
        start += add;
      } else {
        m_wheelSubsystem.setSpinnerTalon(start - add);
        start -= add;
      }
    }

    m_wheelSubsystem.setSpinnerTalon(speed);
    
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
