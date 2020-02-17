package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Add your docs here.
 */
public class ShuffleBoard{

    public boolean update = false;
    public ShuffleboardTab GamePlay = Shuffleboard.getTab("GamePlay");
        public NetworkTableEntry outreachMode = GamePlay.add("outreachMode", false).getEntry();
        public NetworkTableEntry noArm = GamePlay.add("noArm", false).getEntry();
        
    public ShuffleboardTab Tune = Shuffleboard.getTab("Tune");
        private ShuffleboardLayout driveTune = Tune.getLayout("driveTune", BuiltInLayouts.kList);
            public NetworkTableEntry driveType = driveTune.addPersistent("driveType", "c").getEntry(); // a=arcade, c=cheezy, f=field
            public NetworkTableEntry driveSpeed = driveTune.addPersistent("driveSpeed", 0).getEntry();
            public NetworkTableEntry driveSmooth = driveTune.addPersistent("driveSmooth", 0).getEntry();
            public NetworkTableEntry driveFXPidP = driveTune.addPersistent("driveFXPID-P", 0).getEntry();
            public NetworkTableEntry driveFXPidI = driveTune.addPersistent("driveFXPID-I", 0).getEntry();
            public NetworkTableEntry driveFXPidD = driveTune.addPersistent("driveFXPID-D", 0).getEntry();
            public NetworkTableEntry driveFXPidF = driveTune.addPersistent("driveFXPID-F", 0).getEntry();
            public NetworkTableEntry drivePidP = driveTune.addPersistent("drivePID-P", 0).getEntry();
            public NetworkTableEntry drivePidI = driveTune.addPersistent("drivePID-I", 0).getEntry();
            public NetworkTableEntry drivePidD = driveTune.addPersistent("drivePID-D", 0).getEntry();
            public NetworkTableEntry driveTurnPidP = driveTune.addPersistent("driveTurnPID-P", 0).getEntry();
            public NetworkTableEntry driveTurnPidI = driveTune.addPersistent("driveTurnPID-I", 0).getEntry();
            public NetworkTableEntry driveTurnPidD = driveTune.addPersistent("driveTurnPID-D", 0).getEntry();
            public NetworkTableEntry driveSetPosition = driveTune.addPersistent("driveSetPosition", 0).getEntry();
            public NetworkTableEntry driveStraightEffectiveness = driveTune.addPersistent("driveStraightEffectiveness", 0).getEntry();
            public NetworkTableEntry driveLimelightEffectiveness = driveTune.addPersistent("driveLimelightEffectiveness", 0).getEntry();
            public NetworkTableEntry driveArcadeSpeed = driveTune.addPersistent("driveArcadeSpeed", 0).getEntry();
            public NetworkTableEntry driveArcadeTurn = driveTune.addPersistent("driveArcadeTurn", 0).getEntry();

        private ShuffleboardLayout ballTune = Tune.getLayout("ballTune", BuiltInLayouts.kList);
            public NetworkTableEntry ballIntakeSpeed = ballTune.addPersistent("ballIntakeSpeed", .5).getEntry();
            public NetworkTableEntry ballPushSpeed = ballTune.addPersistent("ballPushSpeed", -1).getEntry();
            public NetworkTableEntry ballFlickerInSpeed = ballTune.addPersistent("ballFlickerInSpeed", .5).getEntry();
            public NetworkTableEntry ballFluckerOutIntakeSpeed = ballTune.addPersistent("ballFluckerOutIntakeSpeed", -.5).getEntry();
            public NetworkTableEntry ballBeltsSpeed = ballTune.addPersistent("ballBeltsSpeed", .5).getEntry();
            public NetworkTableEntry ballBeltsOutIntakeSpeed = ballTune.addPersistent("ballBeltsOutIntakeSpeed", -.5).getEntry();
            public NetworkTableEntry ballBeltsOutOutputSpeed = ballTune.addPersistent("ballBeltsOutOutputSpeed", .5).getEntry();
            public NetworkTableEntry ballOutputSpeed = ballTune.addPersistent("ballOutputSpeed", .5).getEntry();
            public NetworkTableEntry ballSpacingWait = ballTune.addPersistent("ballSpacingWait", .5).getEntry();
            public NetworkTableEntry ballSpacingMove = ballTune.addPersistent("ballSpacingMove", .5).getEntry();
            public NetworkTableEntry ball5thSpacingWait = ballTune.addPersistent("ballSpacingWait", .5).getEntry();
            public NetworkTableEntry ball5thSpacingMove = ballTune.addPersistent("ballSpacingMove", .5).getEntry();
            public NetworkTableEntry ballBeltMaxAmp = ballTune.addPersistent("ballBeltMaxAmp", 0).getEntry();

        private ShuffleboardLayout climbTune = Tune.getLayout("climbTune", BuiltInLayouts.kList);
            public NetworkTableEntry climbWinchSpeed = climbTune.addPersistent("climbWinchSpeed", 0).getEntry();  
            public NetworkTableEntry climbElevatorSpeed = climbTune.addPersistent("climbElevatorSpeed", 0).getEntry();  
            public NetworkTableEntry climbDeadZone = climbTune.addPersistent("climbDeadZone", 0).getEntry(); 

        private ShuffleboardLayout wheelTune = Tune.getLayout("wheelTune", BuiltInLayouts.kList);
            public NetworkTableEntry wheelAcceleration = wheelTune.addPersistent("wheelAcceleration", .04).getEntry();
            public NetworkTableEntry wheelRevolutionsMS = wheelTune.addPersistent("wheelRevolutionsMS", 2000).getEntry();   
        
            private ShuffleboardLayout driverTune = Tune.getLayout("driverTune", BuiltInLayouts.kList);
                public NetworkTableEntry driverErrorRumbleLength = driverTune.addPersistent("driverErrorRumbleLength", 0).getEntry();   


    public ShuffleboardTab Debug = Shuffleboard.getTab("Debug");
        private ShuffleboardLayout driveDebug = Debug.getLayout("driveDebug", BuiltInLayouts.kList);
            public NetworkTableEntry driveLeftOutput = driveDebug.add("driveLeftOutput", 0).getEntry();
            public NetworkTableEntry driveRightOutput = driveDebug.add("driveRightOutput", 0).getEntry();
            public NetworkTableEntry driveControlMode = driveDebug.add("driveControlMode", 0).getEntry();
            public NetworkTableEntry driveCurrentType = driveDebug.add("driveCurrentType", "not set!").getEntry();
            public NetworkTableEntry drivePosition = driveDebug.add("drivePosition", "").getEntry();

        private ShuffleboardLayout ballDebug = Debug.getLayout("ballDebug", BuiltInLayouts.kList);
            //controllers
            public NetworkTableEntry ballIntakeOutput = ballDebug.add("ballIntakeOutput", 0).getEntry();
            public NetworkTableEntry ballFlickerOutput = ballDebug.add("ballFlickerOutput", 0).getEntry();
            public NetworkTableEntry ballBeltsOutput = ballDebug.add("ballBeltsOutput", 0).getEntry();
            public NetworkTableEntry ballOutputOutput = ballDebug.add("ballOutputOutput", 0).getEntry();
            //solenoids
            public NetworkTableEntry ballIntakeState = ballDebug.add("ballIntakeState", 0).getEntry();
            public NetworkTableEntry ballLowerBlockerState = ballDebug.add("ballLowerBlockerState", 0).getEntry();
            public NetworkTableEntry ballUpperBlockerState = ballDebug.add("ballUpperBlockerState", 0).getEntry();
            //DIO
            public NetworkTableEntry ballSensorInDIO = ballDebug.add("ballSensorInDIO", false).getEntry();
            public NetworkTableEntry ballSensorOutDIO = ballDebug.add("ballSensorOutDIO", false).getEntry();
            public NetworkTableEntry ballCount = ballDebug.add("ballCount", 0).getEntry();

        private ShuffleboardLayout climbDebug = Debug.getLayout("climbDebug", BuiltInLayouts.kList);
            public NetworkTableEntry climbElevatorOutput = climbDebug.add("climbElevatorOutput", 0).getEntry();
            public NetworkTableEntry climbWinchOutput = climbDebug.add("climbWinchOutput", 0).getEntry();

        private ShuffleboardLayout wheelDebug = Debug.getLayout("wheelDebug", BuiltInLayouts.kList);
            //controllers
            public NetworkTableEntry wheelSpinnerOutput = wheelDebug.add("wheelSpinnerOutput", 0).getEntry();
            //solenoids
            public NetworkTableEntry wheelPushSolenoid = wheelDebug.add("wheelPushSolenoid", 0).getEntry();
            public NetworkTableEntry wheelRetractSolenoid = wheelDebug.add("wheelRetractSolenoid", 0).getEntry();

        private ShuffleboardLayout pdpDebug = Debug.getLayout("pdpDebug", BuiltInLayouts.kList);
            public NetworkTableEntry pdpVoltage = pdpDebug.add("pdpVoltage", 0).getEntry();
            public NetworkTableEntry pdpTotalAmp = pdpDebug.add("pdpTotalAmp", 0).getEntry();
            public NetworkTableEntry pdpTemperature = pdpDebug.add("pdpTemperature", 0).getEntry();

        private ShuffleboardLayout limeLightDebug = Debug.getLayout("limeLightDebug", BuiltInLayouts.kList);
            public NetworkTableEntry limelightX = limeLightDebug.add("limelightX", 0).getEntry();
            public NetworkTableEntry limelightY = limeLightDebug.add("limelightY", 0).getEntry();
            public NetworkTableEntry limelightArea = limeLightDebug.add("limelightArea", 0).getEntry();

    public ShuffleBoard() {

    }
}   
