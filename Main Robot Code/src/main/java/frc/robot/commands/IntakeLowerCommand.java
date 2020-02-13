/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Looper.Loop;
import frc.robot.subsystems.BallSubsystem;

public class IntakeLowerCommand extends CommandBase {
  private BallSubsystem ballSubsystem;
  /**
   * Creates a new IntakeLowerCommand.
   */
  public IntakeLowerCommand(BallSubsystem ballSubsystem, , Solenoid intakeLowerSolenoid) {
    addRequirements(ballSubsystem);
    this.ballSubsystem = ballSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final int timeToPushForward = 1000;
      final int timeBeforeStopping = 1000;
      final int timeBeforeHolingIntakeDown = 1000;
      long currentTime = System.currentTimeMillis();

          intakeLowerSolenoid.set(true);
          intakeLiftSolenoid.set(false);

          if (currentTime + timeToPushForward >= System.currentTimeMillis()) {
            pushForwardLooper.stop();
          }
        @Override public void onStop() {
          intakeLowerSolenoid.set(false);
            intakeLiftSolenoid.set(true);
          waitToStopLooper.start();
        }
      };
      pushForwardLooper = new Looper(pushForward, 50);
      pushForwardLooper.start();

      
      Loop waitToStop = new Loop(){
        @Override public void onStart() {
        }
        @Override public void onLoop() {
          if (currentTime + timeToPushForward + timeBeforeStopping >= System.currentTimeMillis()) {
            intakeLowerSolenoid.set(false);
            intakeLiftSolenoid.set(true);
            waitToStopLooper.stop();
          }
        }
        @Override public void onStop() {
          holdDownLooper.start();
        }
      };
      waitToStopLooper = new Looper(waitToStop, 50);

      Loop holdDown = new Loop(){
        @Override public void onStart() {
        }
        @Override public void onLoop() {
          if (currentTime + timeToPushForward + timeBeforeStopping + timeBeforeHolingIntakeDown >= System.currentTimeMillis()) {
            intakeLowerSolenoid.set(true);
            intakeLiftSolenoid.set(false);
            holdDownLooper.stop();
          }
        }
        @Override public void onStop() {
        }
      };
      holdDownLooper = new Looper(holdDown, 50);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
