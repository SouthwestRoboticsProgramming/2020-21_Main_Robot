package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BallCommand;
import frc.robot.commands.CalibrateGyroCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.ManualDriveCommand.DriveType;
import frc.robot.commands.SetFrontOfRobotCommand;
import frc.robot.commands.WheelCommand;
import frc.robot.commands.WheelCommand.Spin;
import frc.robot.commands.AutoCommands.AutonomonousSelector;
import frc.robot.commands.DriveWithLimelightToHp.WaitBeforeDrivingToHP;
import frc.robot.controllers.Xbox;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
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
  public final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final WheelSubsystem wheelSubsystem = new WheelSubsystem();
  
  private final Xbox controller = new Xbox(Constants.controllerPort);

  private final Joystick XBOX = new Joystick(0);
  private final JoystickButton intake = new JoystickButton(XBOX, 2);
  private final JoystickButton hold = new JoystickButton(XBOX, 3);
  private final JoystickButton outBottom = new JoystickButton(XBOX, 1);
  private final JoystickButton outTop = new JoystickButton(XBOX, 4);
  private final JoystickButton slowCheezy = new JoystickButton(XBOX, 5);
  private final JoystickButton spinWheel = new JoystickButton(XBOX, 8);
  private final JoystickButton slow = new JoystickButton(XBOX, 6);


  private final Command m_autonomousCommand = new AutonomonousSelector(driveTrainSubsystem, ballSubsystem);
  private final Command manualDrive = new ManualDriveCommand(driveTrainSubsystem, DriveType.cheezy);
  private final Command setFrontOfRobot = new SetFrontOfRobotCommand();
  private final Command calibrateGyro = new CalibrateGyroCommand();
  private final Command climb = new ClimbCommand(climbSubsystem);

  public RobotContainer() {
    // Configure the button bindings
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
    intake.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem, ballMode.intake));
    // intake.whenPressed();

    hold.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem,ballMode.hold));
    outBottom.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem,ballMode.unloadIntake));
    outTop.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem,ballMode.unloadOutput));
    // pushBalls.whenPressed(new BallCommand(ballSubsystem, driveTrainSubsystem, ballMode.pushBalls));

    spinWheel.whenPressed(new WheelCommand(wheelSubsystem, driveTrainSubsystem, Spin.Revolutions));
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
      // return xbox.getY(Hand.kLeft);
      return -XBOX.getRawAxis(1) * driveReverse;
    }

    public double getLeftTurn() {
      // return xbox.getX(Hand.kLeft);
      return XBOX.getRawAxis(0);
    }

    public boolean getOneQuickTurn() {
      return false;
    }

    public double getWallEffeciveness() {
      return XBOX.getRawAxis(3);
    }

    public double getLimelightEffeciveness() {
      return XBOX.getRawAxis(2);
    }

    public boolean getSlowCheezy() {
      return XBOX.getRawButton(5);
    }

    public void setDriveReversed(boolean reversed) {
      if (reversed) {
        driveReverse = -1;
      } else {
        driveReverse = 1;
      }
    }
  
    public int getPOV() {
      return XBOX.getPOV();
    }

  // climb
  public double getClimbOutput() {
    return -XBOX.getRawAxis(5);
  }

  public boolean getWinchOutput() {
    return XBOX.getRawButton(10);
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
