package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight{

  public NetworkTable limelightNetworkTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double limelightX() {
    NetworkTableEntry tx = limelightNetworkTable().getEntry("tx");
    return tx.getDouble(0.0);
  }

  public double limelightY() {
    NetworkTableEntry ty = limelightNetworkTable().getEntry("ty");
    return ty.getDouble(0.0);
  }

  public double limelightArea() {
    NetworkTableEntry ta = limelightNetworkTable().getEntry("ta");
    return ta.getDouble(0.0);
  }
  
  public void limelightSetPipeline(int pipeline) {
    NetworkTableEntry pipelineEntry = limelightNetworkTable().getEntry("pipeline");
    pipelineEntry.setNumber(pipeline);
  }

}
