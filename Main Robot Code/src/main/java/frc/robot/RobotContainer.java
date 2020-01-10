package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Controller.Axis;
import frc.lib.Controller.Buttons;
import frc.lib.Controller.Controller;
import frc.lib.Controller.ControllerSet;
import frc.lib.Controller.MappedController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomonousCommand;
import frc.robot.sensors.ShuffleBoard;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final BaseSubsystem m_baseSubsystem = new BaseSubsystem();
  public final DriveTrain driveTrain = new DriveTrain();
  // private final Joystick m_joystick = new Joystick(0);

  private final CommandBase m_autonomousCommand = new AutonomonousCommand();

  private Controller controller = new Controller();
    private ControllerSet xbox = new ControllerSet();
      private MappedController xBox = new MappedController(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain));
    // configure controllers
    configureControllers();

    // Configure the button bindings
    configureButtonBindings();
  }

  public void configureControllers() {
    xbox.addMappedController(xBox);
      xBox.mapAxis(Axis.centerDrive, 1)
      .mapAxis(Axis.turnDrive, 0);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons

    // Connect the buttons to commands
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  // Single joystick drive
  public double oneDrive() {
    return controller.getAxis(Axis.centerDrive);
  }

  public double oneTurn() {
    return controller.getAxis(Axis.turnDrive);
  }

  public boolean oneQuickTurn() {
    return controller.getButton(Buttons.quickTurn).get();
  }
}
