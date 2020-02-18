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
import frc.robot.sensors.ColorSensor;

public class WheelCommand extends CommandBase {
  private final WheelSubsystem m_wheelSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private Looper powerRamper;
  private Spin spin;
  private ColorSensor.Color color;
  private long startTime;
  private boolean finished = false;

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

  public WheelCommand(WheelSubsystem wheelSubsystem, DriveTrainSubsystem driveTrainSubsystem,  Spin spin, ColorSensor.Color color) {
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
    m_wheelSubsystem.setSpinnerTalon(100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (spin == Spin.Revolutions) {
      double spinTime = Robot.shuffleBoard.wheelRevolutionsMS.getDouble(0);
      if (System.currentTimeMillis() - spinTime >= startTime) {finished = true;}
    } else if (spin == Spin.Position) {
      if (getColor(color) == m_wheelSubsystem.getColor()) {finished = true;}
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

  private ColorSensor.Color getColor(ColorSensor.Color color) {
    if (color == ColorSensor.Color.blue) {
        return ColorSensor.Color.red;
    } else if (color == ColorSensor.Color.green) {
      return ColorSensor.Color.yellow;
    } else if (color == ColorSensor.Color.red) {
      return ColorSensor.Color.blue;
    } else if (color == ColorSensor.Color.yellow) {
      return ColorSensor.Color.green;
    } else {
      DriverStation.reportError("Color " + color + " not found!", true);
      return null;
    }
  }
  


}
