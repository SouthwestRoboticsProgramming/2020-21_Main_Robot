package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TCA9548A;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class WheelSubsystem extends SubsystemBase {

  private WPI_TalonSRX spinnerTalon;
  private Solenoid pushSolenoid, retractSolenoid;
  private ADXRS450_Gyro gyro;

  private final int spinnerTalonPort = 0;
  
  private final int pushSolenoidPort = 0,
                    retractSolenoidPort = 0;
  
  public WheelSubsystem() {
    spinnerTalon = new WPI_TalonSRX(spinnerTalonPort);
    pushSolenoid = new Solenoid(pushSolenoidPort);
    retractSolenoid = new Solenoid(retractSolenoidPort);
    gyro = new ADXRS450_Gyro(Port.kMXP);
  }

  //Spinner talon SRX
  public void setSpinnerTalon(double x){
    spinnerTalon.set(x);
  }

  public double getSpinnerTalon(){
    return spinnerTalon.get();
  }
  
  //Push solenoid
  public void setPushSolenoid(boolean x){
    pushSolenoid.set(x);
  }


  //Retract solenoid
  public void setRetractSolenoid(boolean x){
    retractSolenoid.set(x);
  }

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 cs = new ColorSensorV3(i2cPort);
  TCA9548A TCA9548A = new TCA9548A();

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
