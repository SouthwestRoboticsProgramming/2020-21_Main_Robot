/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Add your docs here.
 */
public class TCA9548A {
    int adress = 0x70;
    I2C i2c = new I2C(Port.kOnboard, adress);

    public void transmit(int port, int data) {
        i2c.
        i2c.write(adress, data);
    }



}


 
// void tcaselect(uint8_t i) {
//   if (i > 7) return;
 
//   Wire.beginTransmission(TCAADDR);
//   Wire.write(1 << i);
//   Wire.endTransmission();  
// }