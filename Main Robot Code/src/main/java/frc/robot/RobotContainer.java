package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BallCommand;
import frc.robot.commands.CalibrateGyroCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.ManualDriveCommand.DriveType;
import frc.robot.commands.SetFrontOfRobotCommand;
import frc.robot.commands.WheelCommand;
import frc.robot.commands.WheelCommand.Spin;
import frc.robot.commands.autoCommands.AutonomonousSelector;
import frc.robot.controllers.Xbox;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.subsystems.WheelSubsystem.Color;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.WheelSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private int driveReverse = -1;

  private final BallSubsystem ballSubsystem = new BallSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final WheelSubsystem wheelSubsystem = new WheelSubsystem();
  
  private final Xbox controller = new Xbox(Constants.controllerPort);

  private final Joystick XBOX = new Joystick(0);
  private final Joystick XBOX2 = new Joystick(0);

  private final JoystickButton intakeNoDrop = new JoystickButton(XBOX2, 2);
  private final JoystickButton hold = new JoystickButton(XBOX2, 3);
  private final JoystickButton intake = new JoystickButton(XBOX2, 1);
  private final JoystickButton unloadOutput = new JoystickButton(XBOX2, 4);
  private final JoystickButton unloadIntake = new JoystickButton(XBOX2, 6);
  private final JoystickButton pushBalls = new JoystickButton(XBOX2, 5);

  private final JoystickButton slowCheezy = new JoystickButton(XBOX, 5);
  private final JoystickButton spinWheel = new JoystickButton(XBOX, 8);
  private final JoystickButton alignWheel = new JoystickButton(XBOX, 7);
  private final JoystickButton slow = new JoystickButton(XBOX, 6);

  private final Command m_autonomousCommand = new AutonomonousSelector(driveTrainSubsystem, ballSubsystem);
  private final Command manualDrive = new ManualDriveCommand(driveTrainSubsystem, DriveType.cheezy);
  private final Command setFrontOfRobot = new SetFrontOfRobotCommand();
  private final Command calibrateGyro = new CalibrateGyroCommand();
  private final Command climb = new ClimbCommand(climbSubsystem);

  public RobotContainer() {
    configureButtonBindings();
  }

  public void startTeleopCommands() {
    manualDrive.schedule();
    setFrontOfRobot.schedule();
    calibrateGyro.schedule();
    climb.schedule();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // // Create some buttons
    intakeNoDrop.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem, ballMode.intakeNoDrop));
    hold.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem,ballMode.hold));
    intake.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem,ballMode.intake));
    unloadOutput.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem,ballMode.unloadOutput));
    pushBalls.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem, ballMode.pushBalls));
    // unloadIntake.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem, ballMode.unloadIntake));

    spinWheel.whenPressed(new WheelCommand(wheelSubsystem, driveTrainSubsystem, Spin.Revolutions));
    alignWheel.whenPressed(new WheelCommand(wheelSubsystem, driveTrainSubsystem, Spin.Position, Color.red));
    slow.whenPressed(new ManualDriveCommand(driveTrainSubsystem, DriveType.arcade));
    slow.whenReleased(new ManualDriveCommand(driveTrainSubsystem, DriveType.cheezy));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public edu.wpi.first.wpilibj2.command.Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  // DRIVE
    public double getLeftDrive() {
      return -XBOX.getRawAxis(1) * driveReverse;
    }

    public double getLeftTurn() {
      return XBOX.getRawAxis(0);
    }

    public boolean getOneQuickTurn() {
      return false;
    }

    public double getWallEffectiveness() {
      return 0;
    }

    public double getLimelightEffectiveness() {
      return 0;
    }

    public boolean getSlowCheezy() {
      return slowCheezy.get();
    }

    public double getSpeedMultiplier() {
      double slowestSpeed = .1;
      double output = Math.abs(XBOX.getRawAxis(3) - 1);
      if (output < slowestSpeed) {
        output = slowestSpeed;
      }
      return output;
    }

    public void setDriveReversed(boolean reversed) {
      if (reversed) {
        driveReverse = 1;
      } else {
        driveReverse = -1;
      }
    }
  
    public int getPOV() {
      return XBOX.getPOV();
    }

    // wheel

    public boolean voidTask() {
      return getPOV() == 90;
    }

    public double getWheelManual() {
      return XBOX.getRawAxis(4);
    }

  // climb
  public double getClimbOutput() {
    return -XBOX2.getRawAxis(5);
  }

  public boolean getWinchOutput() {
    return XBOX2.getRawButton(10);
  }

  public double getBothClimb() {
    return XBOX2.getRawAxis(2);
  }

  public boolean getLockButton() {
    return XBOX2.getRawButton(7);
  }

  // ball
  public boolean getIntakeBotton() {
    return intake.get();
  }

  //Other
  public void setRumble(RumbleType type, long ms) {
    controller.rumble(type, ms);
  }

  // Highly important function 
  public <A extends Number> double YEET(A x, A y) {
    return (double) x + (double) y;
  }
}
