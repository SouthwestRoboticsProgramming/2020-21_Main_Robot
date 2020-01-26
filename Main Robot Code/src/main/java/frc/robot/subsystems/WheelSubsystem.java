package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TCA9548A;
import frc.robot.Constants;
import frc.robot.Robot;

/*
* Wheel subsystem includes all motors, solenoids, and sensors assosiated with spinning the wheel of fortune.
*/
public class WheelSubsystem extends SubsystemBase {

  private final WPI_TalonSRX spinnerTalon;
  private final Solenoid pushSolenoid, retractSolenoid;
  private final ADXRS450_Gyro gyro;
  private double gyroOffset = 0;

  private final int spinnerTalonPort = 9;
  
  private final int encoderTicks = 4096;

  public enum Color{
    red,blue,green,yellow, noColor
  } 
  
  public WheelSubsystem() {
    spinnerTalon = new WPI_TalonSRX(spinnerTalonPort);
    pushSolenoid = new Solenoid(Constants.PCMID, Constants.pushSolenoidPort);
    retractSolenoid = new Solenoid(Constants.PCMID, Constants.retractSolenoidPort);
    gyro = new ADXRS450_Gyro(Port.kMXP);
    if (retractSolenoid.get()) {
      gyroOffset = gyro.getAngle()-30;
    } else {
      gyroOffset = gyro.getAngle()-110;
    }

    spinnerTalon.setSelectedSensorPosition(0);
    spinnerTalon.configNeutralDeadband(0.001);
    spinnerTalon.setNeutralMode(NeutralMode.Brake);
    spinnerTalon.setSensorPhase(false);
    spinnerTalon.setInverted(false);

    spinnerTalon.config_kP(0, 0);
    spinnerTalon.config_kI(0, 0);
    spinnerTalon.config_kD(0, 0);
    spinnerTalon.config_kF(0, 0);
  }

  public double getAngle() {
    return gyro.getAngle()-gyroOffset;
  }

  //Spinner talon SRX
  public void setSpinnerTalon(double x){
    spinnerTalon.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.wheelSpinnerOutput.setDouble(spinnerTalon.getMotorOutputPercent());
  }

  public double getSpinnerTalon(){
    return spinnerTalon.get();
  }

  public void moveSpinnerTalon(double rotations){
    spinnerTalon.set(ControlMode.Position, (spinnerTalon.getSelectedSensorPosition() + encoderTicks) * rotations);
  }

  public double getEncoderRevolutions(){
    return spinnerTalon.getSelectedSensorPosition() / encoderTicks;
  }
  
  //Push solenoid 
  public void setPushedState(boolean pushed) {
    pushSolenoid.set(pushed);
    retractSolenoid.set(!pushed);
    Robot.shuffleBoard.wheelPushSolenoid.setBoolean(pushSolenoid.get());
    Robot.shuffleBoard.wheelRetractSolenoid.setBoolean(retractSolenoid.get());
  }

  //Color sensor
  I2C.Port i2cPort = I2C.Port.kOnboard;
  // ColorSensorV3 cs = new ColorSensorV3(i2cPort);
  TCA9548A TCA9548A = new TCA9548A();

  int adress = 0x70;
  // I2C i2c = new I2C(Port.kOnboard, adress);
  // I2C i2cPort = new I2C(Port.kOnboard, adress);
  // ColorSensorV3 cs = new ColorSensorV3(kk);

  public void selectInput(int port, int data) {
      // i2cPort.
  }

  // public Color getColor() {
  //   int r = cs.getRed();
  //   int g = cs.getGreen();
  //   int b = cs.getBlue();

  //   if (.9*g < r && r < 1.9*g && 1.9*b < r && r < 4*b) {
  //     return Color.red;
  //   } else if (2*r < g && g < 5*r && 2*b < g && g < 4*b) {
  //     return Color.green;
  //   } else if (1.5*r < b && b < 3*r && (Math.abs(b+g)/2)/b < .3 * b) {
  //     return Color.blue;
  //   } else if (1.4*r < g && g < 2*r && 3*b < g && g < 5*b) {
  //     return Color.yellow;
  //   } else {
  //     return Color.noColor;
  //   }
  // }

  @Override
  public void periodic() {
    
  }

}
