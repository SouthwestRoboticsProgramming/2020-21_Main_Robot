package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.WheelCommand;
import frc.robot.commands.AutonomonousCommand;
import frc.robot.commands.BallCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.controllers.Xbox;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriverFeedbackSubsystem;
import frc.robot.subsystems.WheelSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private final BallSubsystem ballSubsystem = new BallSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  public final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final WheelSubsystem wheelSubsystem = new WheelSubsystem();
  private final DriverFeedbackSubsystem driverFeedback = new DriverFeedbackSubsystem(this);
  
  private final Xbox controller = new Xbox(Constants.controllerPort);
  private final XboxController xbox = new XboxController(0);

  private final CommandBase m_autonomousCommand = new AutonomonousCommand();
  private final Command manualDrive = new ManualDriveCommand(driveTrainSubsystem);
  private final CommandBase ballSubsystemCommand = new BallCommand(ballSubsystem, ballMode.hold);
  private final CommandBase spinWheel = new WheelCommand(wheelSubsystem, driveTrainSubsystem, driverFeedback, WheelCommand.Spin.Revolutions);
  private final CommandBase positionWheel = new WheelCommand(wheelSubsystem, driveTrainSubsystem, driverFeedback, WheelCommand.Spin.Position);
  private final CommandBase climb = new ClimbCommand(climbSubsystem);

  public RobotContainer() {
    // driveTrainSubsystem.setDefaultCommand(manualDrive); manualDrive.schedule();
    // ballSubsystem.setDefaultCommand(ballSubsystemCommand); ballSubsystemCommand.schedule();
    // climbSubsystem.setDefaultCommand(climb); climb.schedule();

    // Configure the button bindings
    configureButtonBindings();
  }

  public void startTeleopCommands() {
    manualDrive.schedule();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    controller.getButton(Xbox.Button.intake).whenPressed(new BallCommand(ballSubsystem, ballMode.intake));
    controller.getButton(Xbox.Button.hold).whenPressed(new BallCommand(ballSubsystem, ballMode.hold));
    controller.getButton(Xbox.Button.unloadIntake).whenPressed(new BallCommand(ballSubsystem, ballMode.unloadIntake));
    controller.getButton(Xbox.Button.unloadOutput).whenPressed(new BallCommand(ballSubsystem, ballMode.unloadOutput));
    controller.getButton(Xbox.Button.wheelPosition).whenPressed(positionWheel);
    controller.getButton(Xbox.Button.wheelPosition).whenPressed(spinWheel);

    // Connect the buttons to commands
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public edu.wpi.first.wpilibj2.command.Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  //TODO: Do these functions need to be in robotContainer? What are they for?

  // Single joystick drive
  public double getOneDrive() {
    return xbox.getY(Hand.kLeft);
  }

  public double getOneTurn() {
    return -xbox.getX(Hand.kLeft);
  }

  public boolean getOneQuickTurn() {
    return xbox.getBButton();
  }

  public double getWallEffeciveness() {
    return 0;
  }

  public double getLimelightEffeciveness() {
    return 0;
  }

  // Highly important function
  public <A extends Number> double YEET(A x, A y) {
    return (double) x + (double) y;
  }

  // climb
  public double getClimbOutput() {
    return 0;
  }

  public void setRumble(RumbleType type, long ms) {
    controller.rumble(type, ms);
  }
}
