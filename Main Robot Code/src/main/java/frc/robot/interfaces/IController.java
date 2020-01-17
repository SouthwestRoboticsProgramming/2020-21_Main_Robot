/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.interfaces;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * Add your docs here.
 */
public interface IController {
    public double getAnalog(Hand hand, AxisType axis);
    
    default public double getAnalog(AxisType axis) { // for arcadeDrive
        return getAnalog(Hand.kLeft, axis);
    }
    default public double getAnalog(Hand hand) { // for tankDrive
        return getAnalog(hand, AxisType.kY);
    }
    public double getTrigger(Hand hand) ;
    
    public JoystickButton getButton(Button button);

    default public void rumble(double magnitude) {}

    public void defaultDrive(DriveTrainSubsystem train);

    default public void reverse() {};
    
    default public void setReversed(boolean val) {}

    public enum Button {
        intake,
        hold,
        unloadIntake,
        unloadOutput;
    }
}
