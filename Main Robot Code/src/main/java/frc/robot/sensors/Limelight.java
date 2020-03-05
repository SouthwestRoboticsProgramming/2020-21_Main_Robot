package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight{

  public Limelight() {
    limelightNetworkTable().getEntry("stream").setDouble(2);
  }

  public NetworkTable limelightNetworkTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  // Whether the limelight has any valid targets (0 or 1)
  public double getValitTarget() {
    NetworkTableEntry tv = limelightNetworkTable().getEntry("ty");
    return tv.getDouble(0.0);
  }
  
  // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  public double getX() {
    NetworkTableEntry tx = limelightNetworkTable().getEntry("tx");
    return tx.getDouble(0.0);
  }

  // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  public double getY() {
    NetworkTableEntry ty = limelightNetworkTable().getEntry("ty");
    return ty.getDouble(0.0);
  }

  // Target Area (0% of image to 100% of image)
  public double getArea() {
    NetworkTableEntry ta = limelightNetworkTable().getEntry("ta");
    return ta.getDouble(0.0);
  }

  // Skew or rotation (-90 degrees to 0 degrees)
  public double getRotation() {
    NetworkTableEntry ts = limelightNetworkTable().getEntry("ts");
    return ts.getDouble(0.0);
  }

  // Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  public double getHorizontal() {
    NetworkTableEntry thor = limelightNetworkTable().getEntry("thor");
    return thor.getDouble(0.0);
  }

  // Vertical sidelength of the rough bounding box (0 - 320 pixels)
  public double getVertical() {
    NetworkTableEntry tvert = limelightNetworkTable().getEntry("tvert");
    return tvert.getDouble(0.0);
  }


  /** 
	* Sets limelight’s LED state
  * 0 = use the LED Mode set in the current pipeline
  * 1 = force off
  * 2 = force blink
  * 3 = 	force on
	* @param mode ledMode
	*/
  public void setLedMode(int mode) {
    NetworkTableEntry ledMode = limelightNetworkTable().getEntry("ledMode");
    ledMode.setNumber(mode);
  }

  /** 
	* Sets limelight’s operation mode
  * 0 = Vision processor
  * 1 = Driver Camera (Increases exposure, disables vision processing)
	* @param mode camMode
	*/
  public void setCamMode(int mode) {
    NetworkTableEntry camMode = limelightNetworkTable().getEntry("camMode");
    camMode.setNumber(mode);
  }

  /** 
	* Sets limelight’s current pipeline
  * 0..9 = Select pipeline 0..9
	* @param pipeline
	*/
  public void setPipeline(int pipeline) {
    NetworkTableEntry pipe = limelightNetworkTable().getEntry("pipeline");
    pipe.setNumber(pipeline);
  }


}
