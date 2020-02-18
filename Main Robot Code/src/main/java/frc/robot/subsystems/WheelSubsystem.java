package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.ColorSensor.Color;

/*
* Wheel subsystem includes all motors, solenoids, and sensors assosiated with spinning the wheel of fortune.
*/
public class WheelSubsystem extends SubsystemBase {

  private final WPI_TalonSRX spinnerTalon;
  private final DoubleSolenoid wheelDoubleSolenoid;
  private DigitalInput limitSwitch;
  private final ColorSensor colorSensor;
  private Color color;
  // private final int spinnerTalonPort = 9;
  // private final int encoderTicks = 4096;
  
  public WheelSubsystem() {
    spinnerTalon = new WPI_TalonSRX(Constants.wheelOfFortuneTalonPort);
    limitSwitch = new DigitalInput(Constants.wheelOfFortuneLimitPort);
    wheelDoubleSolenoid = new DoubleSolenoid(Constants.PCMID, Constants.pushSolenoidPort, Constants.retractSolenoidPort);
    wheelDoubleSolenoid.set(Value.kOff);

    spinnerTalon.configNeutralDeadband(0.001);
    spinnerTalon.setNeutralMode(NeutralMode.Brake);
    spinnerTalon.setInverted(false);

    spinnerTalon.config_kP(0, 0);
    spinnerTalon.config_kI(0, 0);
    spinnerTalon.config_kD(0, 0);
    spinnerTalon.config_kF(0, 0);

    colorSensor = new ColorSensor(I2C.Port.kOnboard);
  }

  //Spinner talon SRX
  public void setSpinnerTalon(double x){
    spinnerTalon.set(ControlMode.PercentOutput, x);
    Robot.shuffleBoard.wheelSpinnerOutput.setDouble(spinnerTalon.getMotorOutputPercent());
  }

  public double getSpinnerTalon(){
    return spinnerTalon.get();
  }

  public boolean getLimit() {
    return limitSwitch.get();
  }

  
  //Push solenoid 
  public void setPushedState(boolean pushed) {
    if (pushed) {
      wheelDoubleSolenoid.set(Value.kForward);
    }
    else {
      wheelDoubleSolenoid.set(Value.kReverse);
    }
    
    //TODO: Readd shuffleboard
    // Robot.shuffleBoard.wheelPushSolenoid.setBoolean(pushSolenoid.get());
    // Robot.shuffleBoard.wheelRetractSolenoid.setBoolean(retractSolenoid.get());
  }

  public ColorSensor.Color getColor() {
    return color;
  }


  @Override
  public void periodic() {
    // System.out.println("wheel angle = " + gyro.getAngle());
    color = colorSensor.getColor();
  }

}
