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
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriverFeedbackSubsystem;
import frc.robot.subsystems.DriverFeedbackSubsystem.PresetColors;
import frc.robot.subsystems.WheelSubsystem;

public class WheelOfFortune extends CommandBase {
  private WheelSubsystem wheelSubsystem;
  private DriveTrainSubsystem driveTrainSubsystem;
  private DriverFeedbackSubsystem driverFeedback;
  private PID pid = new PID(0, 0, 0);
  private Looper powerRamper;
  private Looper positionHolder;
  private Looper spinner;
  private Spin spin;
  private WheelSubsystem.Color color;
  private final double spinCount = 5;
  private final long loopTime = 20;
  private boolean finished = false;

  public enum Spin {
    Revolutions,
    Position;
  }

  public WheelOfFortune(WheelSubsystem wheelSubsystem, DriveTrainSubsystem driveTrainSubsystem, DriverFeedbackSubsystem driverFeedbackSubsystem, Spin spin) {
    this.wheelSubsystem = wheelSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.driverFeedback = driverFeedbackSubsystem;
    this.spin = spin;
    addRequirements(this.wheelSubsystem);
  }

  public WheelOfFortune(WheelSubsystem wheelSubsystem, DriveTrainSubsystem driveTrainSubsystem, DriverFeedbackSubsystem driverFeedbackSubsystem,  Spin spin, WheelSubsystem.Color color) {
    this.wheelSubsystem = wheelSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.driverFeedback = driverFeedbackSubsystem;
    this.spin = spin;
    this.color = color;
    addRequirements(this.wheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetPoint(90);
    pid.setError(100);    

    while (pid.getError() > 1) {
      driverFeedback.setColor(PresetColors.orange);
      pid.setActual(wheelSubsystem.getAngle());
      driveTrainSubsystem.driveAtAngle(driveTrainSubsystem.getVelocity(pid.getOutput()), 0, ControlMode.Velocity);
    }
    driverFeedback.setColor(PresetColors.green);
    holdPosition();
    wheelSubsystem.setPushedState(true);

    if (spin == Spin.Position) {
      while (getColor(wheelSubsystem.getColor()) != color) {
        rampPower(0, .5);
      }
        rampPower(.5, 0);
    } else if (spin == Spin.Revolutions) {
      Loop loop = new Loop(){
        double setRevolution;
        @Override public void onStart() {
          setRevolution = wheelSubsystem.getEncoderRevolutions() + spinCount;
          wheelSubsystem.moveSpinnerTalon(spinCount);
        }
        @Override public void onLoop() {
          if (wheelSubsystem.getEncoderRevolutions() >= setRevolution) {spinner.stop();}

        }
        @Override public void onStop() {
          wheelSubsystem.setSpinnerTalon(0);
          wheelSubsystem.setPushedState(false);
          finished = false;
        }
      };
      spinner = new Looper(loop, loopTime);
      spinner.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      driverFeedback.setColor(PresetColors.red);
      driverFeedback.errorRumble();
    }
    wheelSubsystem.setPushedState(false);
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
        wheelSubsystem.setSpinnerTalon(start);
      }
      @Override public void onLoop() {
        if (wheelSubsystem.getSpinnerTalon() >= end || finished) {powerRamper.stop();}
        wheelSubsystem.setSpinnerTalon(wheelSubsystem.getSpinnerTalon() + loopAdded);
      }
      @Override public void onStop() {
        wheelSubsystem.setSpinnerTalon(end);
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
        pid.setActual(wheelSubsystem.getAngle());
        driveTrainSubsystem.driveAtAngle(driveTrainSubsystem.getVelocity(pid.getOutput()), 0, ControlMode.Velocity);
      }
      @Override public void onStop() {
      }
    };
    positionHolder = new Looper(loop, loopTime);
    positionHolder.start();
  }


}
