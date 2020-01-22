/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController;
/**
 * Add your docs here.
 */
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.TimeOutTimer;
import frc.lib.Looper.Loop;
import frc.lib.Looper.Looper;
import frc.robot.interfaces.IController;
import frc.robot.subsystems.DriveTrainSubsystem;;
public class Xbox implements IController {
    final int PORT;
    private XboxController xbox;
    private double reverseVal = 1;
    private Looper rumbleLooper;

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
            case wheelSpin : return new JoystickButton(xbox, XboxMap.Select.toInt());
            case wheelPosition : return new JoystickButton(xbox, XboxMap.Windows.toInt());
            default         : return new JoystickButton(xbox, XboxMap.X.toInt());
        }
    }
    
    public void rumble(RumbleType type, long ms) {
        TimeOutTimer timer = new TimeOutTimer(ms);
        Loop loop = new Loop(){
            @Override public void onStart() {
                xbox.setRumble(type, 1);
                timer.start();
            }
            @Override public void onLoop() {
              if (timer.getTimedOut()) {rumbleLooper.stop();}
            }
            @Override public void onStop() {
                timer.stop();
            }
          };
          rumbleLooper = new Looper(loop, 100);
          rumbleLooper.start();
        
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
