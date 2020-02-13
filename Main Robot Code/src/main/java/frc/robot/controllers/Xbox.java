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
public class Xbox implements Controller {
    final int PORT;
    private XboxController xbox;

    public Xbox(int port) {
        PORT = port;
        xbox = new XboxController(PORT);
		init();
    }
	final Control<R<Doub>> drive = new Control<R<Doub>>() {
		public boolean get() {
			return true;
		}
		public R<Doub> out() {
			return new R<Doub>(xbox.getX(Hand.kLeft), xbox.getY(Hand.kLeft) );
		}
	}
	
	

    
    public void rumble(double intensity) {
	// something;
	}
	public void lights(int id, boolean state) {
	// put stuff here
	}
	
 }
