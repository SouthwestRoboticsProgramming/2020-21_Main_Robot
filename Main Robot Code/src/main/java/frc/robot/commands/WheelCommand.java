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
import frc.lib.PID;
import frc.lib.Looper.Loop;
import frc.lib.Looper.Looper;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriverFeedbackSubsystem;
import frc.robot.subsystems.DriverFeedbackSubsystem.PresetColors;
import frc.robot.subsystems.WheelSubsystem;

public class WheelCommand extends CommandBase {
  private final WheelSubsystem m_wheelSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private DriverFeedbackSubsystem driverFeedback = Robot.driverFeedback;
  private PID pid;
  private Looper powerRamper;
  private Looper positionHolder;
  private Looper spinner;
  private Spin spin;
  private boolean pressure = false;
  private WheelSubsystem.Color color;
  private long startTime;
  private final double spinTime = 1000;
  private final long loopTime = 20;
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
    m_wheelSubsystem.calibrate();
    System.out.println("WheelCommand.initialize()");
    startTime = System.currentTimeMillis();
    m_wheelSubsystem.setPushedState(true);

    pid = new PID(0, 0, 0);
    pid.setSetPoint(70);
    pid.setError(70);    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("WheelCommand.execute()");
    if (System.currentTimeMillis() - spinTime >= startTime) {finished = true;}

    if (pid.getError() < 2) {
      m_wheelSubsystem.setSpinnerTalon(50);
    }

    pid.setActual(m_wheelSubsystem.getAngle());
    driveTrainSubsystem.driveAtAngle(pid.getOutput(), 0, ControlMode.Velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WheelCommand.end()");
    if (interrupted) {
      driverFeedback.setColor(PresetColors.red);
      // driverFeedback.errorRumble();
    }
    m_wheelSubsystem.setSpinnerTalon(0);
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
      DriverStation.reportError("Color " + color + " not found!", true);
      return null;
    }
  }

  private void rampPower(double start, double end) {
    double loopAdded = .04;
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

  private void holdPosition() {
    Loop loop = new Loop(){
      @Override public void onStart() {

      }
      @Override public void onLoop() {
        if (finished) {positionHolder.stop();}
        
      }
      @Override public void onStop() {
      }
    };
    positionHolder = new Looper(loop, loopTime);
    positionHolder.start();
  }


}
