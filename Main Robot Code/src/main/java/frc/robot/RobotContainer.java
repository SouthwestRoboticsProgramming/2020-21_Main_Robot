package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.*;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.WheelSubsystem;
import frc.robot.subsystems.BallSubsystem.ballMode;
import frc.robot.controllers.Xbox;
import frc.robot.interfaces.IController.Button;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private final BallSubsystem ballSubsystem = new BallSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final WheelSubsystem wheelSubsystem = new WheelSubsystem();
  
  private final Xbox controller = new Xbox(Constants.controllerPort);;

  private final CommandBase m_autonomousCommand = new AutonomonousCommand();
  private final CommandBase arcadeDrive = new ArcadeDrive(driveTrainSubsystem);
  private final CommandBase ballSubsystemCommand = new BallSubsystemCommand(ballSubsystem, ballMode.hold);

  // private Controller controller = new Controller();
  //   private ControllerSet xbox = new ControllerSet();
  //     private MappedController xBox = new MappedController(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveTrainSubsystem.setDefaultCommand(arcadeDrive);
    ballSubsystem.setDefaultCommand(ballSubsystemCommand);

    arcadeDrive.schedule();
    ballSubsystemCommand.schedule();

    // configure controllers
    configureControllers();

    // Configure the button bindings
    configureButtonBindings();
  }

  public void configureControllers() {
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    controller.getButton(Xbox.Button.intake).whenPressed(new BallSubsystemCommand(ballSubsystem, ballMode.intake));
    controller.getButton(Xbox.Button.hold).whenPressed(new BallSubsystemCommand(ballSubsystem, ballMode.hold));
    controller.getButton(Xbox.Button.unloadIntake).whenPressed(new BallSubsystemCommand(ballSubsystem, ballMode.unloadIntake));
    controller.getButton(Xbox.Button.unloadOutput).whenPressed(new BallSubsystemCommand(ballSubsystem, ballMode.unloadOutput));

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

  // Single joystick drive
  public double oneDrive() {
    return 0;
  }

  public double oneTurn() {
    return 0;
  }

  public boolean oneQuickTurn() {
    return false;
  }
}
