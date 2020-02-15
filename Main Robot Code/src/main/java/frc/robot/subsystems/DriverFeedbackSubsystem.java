/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriverFeedbackSubsystem extends SubsystemBase {
  final RobotContainer robotContainer;
  final int controllerPWMChannel = 0;
  final Spark controller;

  public DriverFeedbackSubsystem(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    controller = new Spark(controllerPWMChannel);
  }

  public enum PresetColors {
    red(1), orange(1), green(1), purple(1);
    PresetColors(double PresetColors){value = PresetColors;}
    double value;
    public double get(PresetColors color){
			return value;
		}
  }
  // faded color values
  static double[] redGreenV = new double[] {1,2,3,4};

  public enum FadedColor {
    redGreen(redGreenV);
    FadedColor(double[] FadedColors){fadedColors = FadedColors;}
    double[] fadedColors;
    public double[] get(FadedColor color){
			return fadedColors;
		}
  }

  private double getFadedColorsValue(FadedColor fadedColor, double percent) {
    double[] color = fadedColor.get(fadedColor);
    return fadedColor.get(fadedColor)[(int)Math.round(color.length * percent)];
  }

  private void setRawColor(double output) {
    // controller.set(output);
  }

  public void setColor(PresetColors color) {
    // setRawColor(color.get(color));
  }

  public void fadeRawColors(PresetColors firstColor, PresetColors secondColor, double percent) {
    // setRawColor(((firstColor.get(firstColor) - secondColor.get(secondColor))*percent) + firstColor.get(firstColor));
  }

  public void fadeColors(FadedColor color, double percent) {
    // getFadedColorsValue(color, percent);
  }

  public void errorRumble() {
    // setRumble(RumbleType.kLeftRumble, (Long)Robot.shuffleBoard.driverErrorRumbleLength.getNumber(0));
  }

  public void setRumble(RumbleType type, long ms) {
    // robotContainer.setRumble(type, ms);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
