package frc.robot.sensors;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PDP {
  PowerDistributionPanel pdp = new PowerDistributionPanel();
  public double PDPVoltage() {
    double voltage = pdp.getVoltage();
    return voltage;
  }

  public double PDPTotalAmp() {
    double totalAmp = pdp.getTotalCurrent();
    return totalAmp;
  } 

  public double PDPBeltAmp() {
    double beltAmp = pdp.getCurrent(12);
    return beltAmp;
  }

  public double PDPLeftDriveAmp() {
    double left1Amp = pdp.getCurrent(0);
    double left2Amp = pdp.getCurrent(1);
    double left3Amp = pdp.getCurrent(2);
    double leftAmp = left1Amp + left2Amp + left3Amp;
    return leftAmp;
  }

  public double PDPRightDriveAmp() {
    double right1Amp = pdp.getCurrent(13);
    double right2Amp = pdp.getCurrent(14);
    double right3Amp = pdp.getCurrent(15);
    double rightAmp = right1Amp + right2Amp + right3Amp;
    return rightAmp;
  }

  public double PDPTemperature() {
    double temperature = pdp.getTemperature();
    return temperature;
  }
}
