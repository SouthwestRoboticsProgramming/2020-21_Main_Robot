/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  
  WPI_TalonSRX winch, elevator;

  private int winchTalonPort = 0, 
              elevatorTalonPort = 0;

  public ClimbSubsystem() {
    winch = new WPI_TalonSRX(winchTalonPort);
    elevator = new WPI_TalonSRX(elevatorTalonPort);

  }

}
