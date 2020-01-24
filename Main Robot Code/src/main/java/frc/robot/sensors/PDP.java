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

  public double PDPWristAmp() {
    double armAmp = pdp.getCurrent(13);
    return armAmp;
  }

  public double PDPArmAmp() {
    double armAmp = pdp.getCurrent(14);
    return armAmp;
  }

  public double PDPExtentionAmp() {
    double armAmp = pdp.getCurrent(15);
    return armAmp;
  }

  public double PDPLeftDriveAmp() {
    double left1Amp = pdp.getCurrent(2);
    double left2Amp = pdp.getCurrent(3);
    double leftAmp = left1Amp + left2Amp;
    return leftAmp;
  }

  public double PDPRightDriveAmp() {
    double right1Amp = pdp.getCurrent(0);
    double right2Amp = pdp.getCurrent(1);
    double rightAmp = right1Amp + right2Amp;
    return rightAmp;
  }

  public double PDPTemperature() {
    double temperature = pdp.getTemperature();
    return temperature;
  }
}
