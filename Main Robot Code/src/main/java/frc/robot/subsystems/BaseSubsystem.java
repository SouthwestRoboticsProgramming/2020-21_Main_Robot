/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BaseSubsystem extends SubsystemBase {
  /**
   * The DriveTrain subsystem incorporates the sensors and actuators attached to the robots chassis.
   * These include four drive motors, a left and right encoder and a gyro.
   */

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 cs = new ColorSensorV3(i2cPort);

  int adress = 0x70;
  // I2C i2c = new I2C(Port.kOnboard, adress);
  // I2C i2cPort = new I2C(Port.kOnboard, adress);
  // ColorSensorV3 cs = new ColorSensorV3(kk);

  public void selectInput(int port, int data) {
      // i2cPort.
  }

  @Override
  public void periodic() {

    // int r = cs.getRed();
    // int g = cs.getGreen();
    // int b = cs.getBlue();

    // String color = "";
    // if (.9*g < r && r < 1.9*g && 1.9*b < r && r < 4*b) {
    //   color = "red";
    // } else if (2*r < g && g < 5*r && 2*b < g && g < 4*b) {
    //   color = "green";
    // } else if (1.5*r < b && b < 3*r && (Math.abs(b+g)/2)/b < .3 * b) {
    //   color = "blue";
    // } else if (1.4*r < g && g < 2*r && 3*b < g && g < 5*b) {
    //   color = "yellow";
    // }
    // System.out.println("red = " + r + " green = " + g + " blue = " + b + " color = " + color);
  }

}
