/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;
import frc.robot.interfaces.IController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
/**
 * Add your docs here.
 */
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;;
public class Xbox implements IController {
    final int PORT;
    private XboxController xbox;
    private double reverseVal = 1;

    public Xbox(int port) {
        PORT = port;
        xbox = new XboxController(PORT);
    }

    public void defaultDrive(DriveTrainSubsystem train) {
        
    }

    public double getAnalog(Hand hand, AxisType axis) {
        double output = 0;
        switch(axis) {
            case kX : output = xbox.getX(hand); break;
            case kY : output = xbox.getY(hand); break;
        }
        return output * reverseVal;
    }
    public JoystickButton getButton(Button button) {
        switch(button) {
            case intake : return new JoystickButton(xbox, XboxMap.A.toInt());
            case hold : return new JoystickButton(xbox, XboxMap.B.toInt());
            case unloadIntake : return new JoystickButton(xbox, XboxMap.X.toInt());
            case unloadOutput : return new JoystickButton(xbox, XboxMap.Y.toInt());
            default         : return new JoystickButton(xbox, XboxMap.X.toInt());
        }
    }
    
    public void rumble(double amount) {
        xbox.setRumble(RumbleType.kLeftRumble, amount);
        xbox.setRumble(RumbleType.kRightRumble,amount);
    }
    public double getTrigger(Hand hand) {
        return xbox.getTriggerAxis(hand);
    }
	private enum XboxMap {
		A(1),
		B(2),
		X(3),
		Y(4),
		LeftTrigger(5),
		RightTrigger(6),
		Windows(7),
		Select(8),
		LeftJoystick(9),
		RightJoystick(10);
		private int num;
		XboxMap(int buttonNumber)
		{
			num = buttonNumber;
		}
		public int toInt()
		{
			return num;
		}
    }
    public void reverse() {
        reverseVal = -Math.signum(reverseVal);
    }
}
