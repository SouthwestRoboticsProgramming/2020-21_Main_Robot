package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.ColorShim;
import com.revrobotics.ColorSensorV3;


public class ColorSensor{

    // private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 cs;
    private final I2C.Port address;
    public enum Color{
        red,blue,green,yellow, noColor
     }
    
    public ColorSensor(I2C.Port address) {
        this.address = address;
        this.cs = new ColorSensorV3(address);
    }
    public Color getColor() {
        int r = cs.getRed();
        int g = cs.getGreen();
        int b = cs.getBlue();
    
        if (.9*g < r && r < 1.9*g && 1.9*b < r && r < 4*b) {
          return Color.red;
        } else if (2*r < g && g < 5*r && 2*b < g && g < 4*b) {
          return Color.green;
        } else if (1.5*r < b && b < 3*r && (Math.abs(b+g)/2)/b < .3 * b) {
          return Color.blue;
        } else if (1.4*r < g && g < 2*r && 3*b < g && g < 5*b) {
          return Color.yellow;
        } else {
          return Color.noColor;
        }
      }
  
}
