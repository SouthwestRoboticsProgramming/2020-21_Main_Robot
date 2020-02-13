/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;

public class IntakeLowerCommand extends CommandBase {
  private DoubleSolenoid intakeDoubleSolenoid;
  private boolean stop = false;
  private long startTime;
  /**
   * Creates a new IntakeLowerCommand.
   */
  public IntakeLowerCommand(BallSubsystem ballSubsystem, DoubleSolenoid intakeDoubleSolenoid) {
    addRequirements(ballSubsystem);
    this.intakeDoubleSolenoid = intakeDoubleSolenoid;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   startTime = System.currentTimeMillis();;
   intakeDoubleSolenoid.set(Value.kForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // final long timeToPushForward = 1000;
      //   boolean pushedForward = false;
      // final long timeBeforeStopping = 1000;
      //   boolean stopped = false;
      // final long timeBeforeHolingIntakeDown = 1000;
      //   boolean heldOut = false;
      

      //     intakeDoubleSolenoid.set(Value.kForward);
      //     if (startTime + timeToPushForward >= System.currentTimeMillis() && !pushedForward) {
      //       intakeDoubleSolenoid.set(Value.kReverse);
      //       pushedForward = true;
      //     }

      //     if (startTime + timeToPushForward + timeBeforeStopping >= System.currentTimeMillis() && !stopped) {
      //       intakeDoubleSolenoid.set(Value.kForward);
      //       stopped = true;
      //       stop = true;
      //     }

      //     // if (startTime + timeToPushForward + timeBeforeStopping + timeBeforeHolingIntakeDown>= System.currentTimeMillis() && !heldOut) {
      //     //   intakeDoubleSolenoid.set(Value.kReverse);
      //     //   heldOut = true;
      //     //   stop = true;
      //     // }
          
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
