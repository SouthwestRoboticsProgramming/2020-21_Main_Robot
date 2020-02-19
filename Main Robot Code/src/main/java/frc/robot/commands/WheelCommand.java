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
import frc.robot.sensors.ColorSensor.Color;

public class WheelCommand extends CommandBase {
  private final WheelSubsystem m_wheelSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private Looper powerRamper;
  private Spin spin;
  private ColorSensor.Color goalColor;
  private ColorSensor.Color initialColor;
  private ColorSensor.Color adjacentColor;
  private boolean onInitialColor;
  private boolean recentlyOnInitial;
  private int rotationsCount;
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
    this.goalColor = color;
    addRequirements(this.m_wheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WheelCommand.initialize()");
    startTime = System.currentTimeMillis();
    m_wheelSubsystem.setPushedState(true);
    finished = false; 

    while(m_wheelSubsystem.getLimit()) {
      driveTrainSubsystem.driveAtAngle(.25, 0, ControlMode.PercentOutput);
    }
<<<<<<< HEAD
    driveTrainSubsystem.stop();
    m_wheelSubsystem.setSpinnerTalon(100);
    initialColor = m_wheelSubsystem.getColor();
    rotationsCount = 0;
    onInitialColor = true;
    recentlyOnInitial = true;
=======
    driveTrainSubsystem.driveMotors(0, 0);
    rampSpinnerPower(0, 100);
>>>>>>> parent of b1da761... added  auto
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
    //TODO: is time the best measure of spins completed? What if power is low? Could color tracking work?
    if (spin == Spin.Revolutions) {
      // double spinTime = Robot.shuffleBoard.wheelRevolutionsMS.getDouble(0);
      // if (System.currentTimeMillis() - spinTime >= startTime) {finished = true;}
      if (m_wheelSubsystem.getColor() == initialColor) {
        if (!onInitialColor) {
          //To avoid counting rotation from being bumped back onto initialColor
          if (!recentlyOnInitial) {
            rotationsCount += 1;
          }
          onInitialColor = true;
          recentlyOnInitial = true;
        }
      }
      else {
        onInitialColor = false;
        // Sets up adjacentColor first time
        if (adjacentColor == null && m_wheelSubsystem.getColor() != Color.noColor) {
          adjacentColor = m_wheelSubsystem.getColor();
        }
        // Sets recentlyOnInitial to false if wheel has passed adjacent color
        if (m_wheelSubsystem.getColor() != adjacentColor && m_wheelSubsystem.getColor() != Color.noColor) {
          recentlyOnInitial = false;
        }
      }
      // Finishes if has rotated three times
      if (rotationsCount == 3) {
        finished = true;
      }
    } else if (spin == Spin.Position) {
      if (getReverseColor(goalColor) == m_wheelSubsystem.getColor()) {finished = true;}
=======
    System.out.println("WheelCommand.execute()");
    if (spin == Spin.Revolutions) {
      double spinTime = Robot.shuffleBoard.wheelRevolutionsMS.getDouble(0);
      if (System.currentTimeMillis() - spinTime >= startTime) {finished = true;}
    } else if (spin == Spin.Position) {
      if (getColor(color) == m_wheelSubsystem.getColor()) {finished = true;}
>>>>>>> parent of b1da761... added  auto
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WheelCommand.end()");
    m_wheelSubsystem.setSpinnerTalon(0);
    new Lib().sleep(1000);
    m_wheelSubsystem.setPushedState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  // Takes in a measured color, returns the color it corresponds to on the field sensor.
  private ColorSensor.Color getReverseColor(ColorSensor.Color color) {
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

  private void rampSpinnerPower(double start, double end) {
    double loopAdded = Robot.shuffleBoard.wheelAcceleration.getDouble(0);
    Long diff = Double.doubleToLongBits((end - start) * 100 / (loopAdded * 100));
    
    Loop loop = new Loop(){
      @Override public void onStart() {
        m_wheelSubsystem.setSpinnerTalon(start);
      }
      @Override public void onLoop() {
        if (m_wheelSubsystem.getSpinnerTalon() >= end || finished) {powerRamper.stop();}
        m_wheelSubsystem.setSpinnerTalon(m_wheelSubsystem.getSpinnerTalon() + loopAdded);
      }
      @Override public void onStop() {
        m_wheelSubsystem.setSpinnerTalon(end);
      }
    };
    powerRamper = new Looper(loop, diff);
    powerRamper.start();
  }

  


}
