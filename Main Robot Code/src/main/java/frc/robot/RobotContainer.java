/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Controller.Axis;
import frc.lib.Controller.Buttons;
import frc.lib.Controller.Controller;
import frc.lib.Controller.ControllerSet;
import frc.lib.Controller.MappedController;
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
  private final CommandBase m_autonomousCommand = new AutonomonousCommand();
  private static BaseSubsystem m_baseSubsystem = new BaseSubsystem();
  public static ShuffleBoard shuffleBoard = new ShuffleBoard();
  public static DriveTrain driveTrain = new DriveTrain();

  private Controller controller = new Controller();
    private ControllerSet xbox = new ControllerSet();
      private MappedController xBox = new MappedController(0);

  

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // setup controllers

    // Configure the button bindings
    configureButtonBindings();
  }

  public void configureControllers() {
    xbox.addMappedController(xBox);
      xBox.mapAxis(Axis.centerDrive, 1)
      .mapAxis(Axis.turnDrive, 0);
  }

  private void configureButtonBindings() {
    // Connect the buttons to commands
  }

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
