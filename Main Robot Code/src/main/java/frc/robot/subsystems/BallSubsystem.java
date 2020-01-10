/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class BallSubsystem extends SubsystemBase {
  /**
   * The ball subsystem includes the intake motors and solenoids, three belt motors, and the output motor and solenoid.
   * It also includes two sensors used for counting stored balls.
   * 
   */

  private WPI_TalonSRX intakeTalon, beltTalon1, beltTalon2, beltTalon3, outputTalon;
  private Solenoid intakeSolenoid1, intakeSolenoid2, outputSolenoid;
  private DigitalInput ballSensorIn, ballSensorOut;
  
}
