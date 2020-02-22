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
        public NetworkTableEntry path1 = GamePlay.add("path1", "").getEntry();
        public NetworkTableEntry path2 = GamePlay.add("path2", "").getEntry();
        public NetworkTableEntry path3 = GamePlay.add("path3", "").getEntry();
        public NetworkTableEntry path4 = GamePlay.add("path4", "").getEntry();
        
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
            public NetworkTableEntry driveTurnPidF = driveTune.addPersistent("driveTurnPID-F", 0).getEntry();
            public NetworkTableEntry driveSetPosition = driveTune.addPersistent("driveSetPosition", 0).getEntry();
            public NetworkTableEntry driveStraightEffectiveness = driveTune.addPersistent("driveStraightEffectiveness", 0).getEntry();
            public NetworkTableEntry driveLimelightEffectiveness = driveTune.addPersistent("driveLimelightEffectiveness", 0).getEntry();
            public NetworkTableEntry driveArcadeSpeed = driveTune.addPersistent("driveArcadeSpeed", 0).getEntry();
            public NetworkTableEntry driveArcadeTurn = driveTune.addPersistent("driveArcadeTurn", 0).getEntry();
            public NetworkTableEntry driveCheezyHSensitivity = driveTune.addPersistent("driveCheezyHSensitivity", 0).getEntry();
            public NetworkTableEntry driveCheezyLSensitivity = driveTune.addPersistent("driveCheezyLSensitivity", 0).getEntry();


        private ShuffleboardLayout ballTune = Tune.getLayout("ballTune", BuiltInLayouts.kList);
            public NetworkTableEntry ballIntakeSpeed = ballTune.addPersistent("ballIntakeSpeed", .5).getEntry();
            public NetworkTableEntry ballPushSpeed = ballTune.addPersistent("ballPushSpeed", -1).getEntry();
            public NetworkTableEntry ballPushHoldSpeed = ballTune.addPersistent("ballPushHoldSpeed", .1).getEntry();
            public NetworkTableEntry ballFlickerInSpeed = ballTune.addPersistent("ballFlickerInSpeed", .5).getEntry();
            public NetworkTableEntry ballFluckerOutIntakeSpeed = ballTune.addPersistent("ballFluckerOutIntakeSpeed", -.5).getEntry();
            public NetworkTableEntry ballBeltsSpeed = ballTune.addPersistent("ballBeltsSpeed", .5).getEntry();
            public NetworkTableEntry ballBeltsOutIntakeSpeed = ballTune.addPersistent("ballBeltsOutIntakeSpeed", -.5).getEntry();
            public NetworkTableEntry ballBeltsOutOutputSpeed = ballTune.addPersistent("ballBeltsOutOutputSpeed", .5).getEntry();
            public NetworkTableEntry ballOutputSpeed = ballTune.addPersistent("ballOutputSpeed", .5).getEntry();
            public NetworkTableEntry ballSpacingWait = ballTune.addPersistent("ballSpacingWait", .5).getEntry();
            public NetworkTableEntry ballSpacingMove = ballTune.addPersistent("ballSpacingMove", .5).getEntry();
            public NetworkTableEntry ball5thSpacingWait = ballTune.addPersistent("ball5thSpacingWait", .5).getEntry();
            public NetworkTableEntry ball5thSpacingMove = ballTune.addPersistent("ball5thSpacingMove", .5).getEntry();
            public NetworkTableEntry ballBeltMaxAmp = ballTune.addPersistent("ballBeltMaxAmp", 0).getEntry();

        private ShuffleboardLayout climbTune = Tune.getLayout("climbTune", BuiltInLayouts.kList);
            public NetworkTableEntry climbWinchSpeed = climbTune.addPersistent("climbWinchSpeed", 0).getEntry();  
            public NetworkTableEntry climbElevatorSpeed = climbTune.addPersistent("climbElevatorSpeed", 0).getEntry();  
            public NetworkTableEntry climbDeadZone = climbTune.addPersistent("climbDeadZone", 0).getEntry(); 

        private ShuffleboardLayout wheelTune = Tune.getLayout("wheelTune", BuiltInLayouts.kList);
            public NetworkTableEntry wheelAcceleration = wheelTune.addPersistent("wheelAcceleration", .04).getEntry();
            public NetworkTableEntry wheelRevolutionsMS = wheelTune.addPersistent("wheelRevolutionsMS", 2000).getEntry();  
            public NetworkTableEntry wheelBackupTimems = wheelTune.addPersistent("wheelBackupTimems", 200).getEntry(); 
        
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

    public ShuffleboardTab Auto = Shuffleboard.getTab("Auto");
        private ShuffleboardLayout autoTune = Auto.getLayout("autoTune", BuiltInLayouts.kList);
            public NetworkTableEntry autoTuneAccelDistence = autoTune.addPersistent("autoTuneAccelDistence", 0).getEntry();
            public NetworkTableEntry autoTuneAcceleration = autoTune.addPersistent("autoTuneAcceleration", 0).getEntry();
            public NetworkTableEntry followWithLimelight = autoTune.addPersistent("followWithLimelight", 0).getEntry();

        private ShuffleboardLayout PathA = Auto.getLayout("PathA", BuiltInLayouts.kList);
            public NetworkTableEntry PathAWait0 = PathA.addPersistent("PathAWait0", 0).getEntry();
            public NetworkTableEntry PathADistence1 = PathA.addPersistent("PathADistence1", 0).getEntry();
            public NetworkTableEntry PathASpeed1 = PathA.addPersistent("PathASpeed1", 0).getEntry();
            public NetworkTableEntry PathATime2 = PathA.addPersistent("PathATime2", 0).getEntry();
            public NetworkTableEntry PathASpeed2 = PathA.addPersistent("PathASpeed2", 0).getEntry();
            public NetworkTableEntry PathAWait3 = PathA.addPersistent("PathAWait3", 0).getEntry();
        
        private ShuffleboardLayout PathB = Auto.getLayout("PathB", BuiltInLayouts.kList);
            public NetworkTableEntry PathBWait0 = PathB.addPersistent("PathBWait0", 0).getEntry();
            public NetworkTableEntry PathBDistence1 = PathB.addPersistent("PathBDistence1", 0).getEntry();
            public NetworkTableEntry PathBSpeed1 = PathB.addPersistent("PathBSpeed1", 0).getEntry();
            public NetworkTableEntry PathBAngle2 = PathB.addPersistent("PathBAngle2", 0).getEntry();
            public NetworkTableEntry PathBDistence3 = PathB.addPersistent("PathBDistence3", 0).getEntry();
            public NetworkTableEntry PathBSpeed3 = PathB.addPersistent("PathBSpeed3", 0).getEntry();
            public NetworkTableEntry PathBAngle4 = PathB.addPersistent("PathBAngle4", 0).getEntry();
            public NetworkTableEntry PathBTime5 = PathB.addPersistent("PathBTime5", 0).getEntry();
            public NetworkTableEntry PathBSpeed5 = PathB.addPersistent("PathBSpeed5", 0).getEntry();
            public NetworkTableEntry PathBWait6 = PathB.addPersistent("PathBWait6", 0).getEntry();
        
        private ShuffleboardLayout PathBNew = Auto.getLayout("PathBNew", BuiltInLayouts.kList);
            public NetworkTableEntry PathBnWait0 = PathBNew.addPersistent("PathBnWait0", 0).getEntry();
            public NetworkTableEntry PathBnAngle1 = PathBNew.addPersistent("PathBnAngle1", 0).getEntry();
            public NetworkTableEntry PathBnDistence2 = PathBNew.addPersistent("PathBnDistence2", 0).getEntry();
            public NetworkTableEntry PathBnSpeed2 = PathBNew.addPersistent("PathBnSpeed2", 0).getEntry();
            public NetworkTableEntry PathBnAngle3 = PathBNew.addPersistent("PathBnAngle3", 0).getEntry();
            public NetworkTableEntry PathBnTime4 = PathBNew.addPersistent("PathBnTime4", 0).getEntry();
            public NetworkTableEntry PathBnSpeed4 = PathBNew.addPersistent("PathBnSpeed4", 0).getEntry();
            public NetworkTableEntry PathBnWait5 = PathBNew.addPersistent("PathBnWait5", 0).getEntry();

        private ShuffleboardLayout PathC = Auto.getLayout("PathC", BuiltInLayouts.kList);
            public NetworkTableEntry PathCAngle1 = PathC.addPersistent("PathCAngle1", 0).getEntry();
            public NetworkTableEntry PathCDistence2 = PathC.addPersistent("PathCDistence2", 0).getEntry();
            public NetworkTableEntry PathCSpeed2 = PathC.addPersistent("PathCSpeed2", 0).getEntry();
            public NetworkTableEntry PathCAngle3 = PathC.addPersistent("PathCAngle3", 0).getEntry();

        private ShuffleboardLayout PathD = Auto.getLayout("PathD", BuiltInLayouts.kList);
            public NetworkTableEntry PathDDistence1 = PathD.addPersistent("PathDDistence1", 0).getEntry();
            public NetworkTableEntry PathDSpeed1 = PathD.addPersistent("PathDSpeed1", 0).getEntry();
            public NetworkTableEntry PathDLimelightTime2 = PathD.addPersistent("PathDLimelightTime2", 0).getEntry();
            public NetworkTableEntry PathDLimelightSpeed2 = PathD.addPersistent("PathDLimelightSpeed2", 0).getEntry();
            public NetworkTableEntry PathDAngle3 = PathD.addPersistent("PathDAngle3", 0).getEntry();
            public NetworkTableEntry PathDDistence4 = PathD.addPersistent("PathDDistence4", 0).getEntry();
            public NetworkTableEntry PathDSpeed4 = PathD.addPersistent("PathDSpeed4", 0).getEntry();

        private ShuffleboardLayout PathE = Auto.getLayout("PathE", BuiltInLayouts.kList);
            public NetworkTableEntry PathEAngle1 = PathE.addPersistent("PathEAngle1", 0).getEntry();
            public NetworkTableEntry PathEDistence2 = PathE.addPersistent("PathEDistence2", 0).getEntry();
            public NetworkTableEntry PathESpeed2 = PathE.addPersistent("PathESpeed2", 0).getEntry();
            public NetworkTableEntry PathEAngle3 = PathE.addPersistent("PathEAngle3", 0).getEntry();

        private ShuffleboardLayout PathF = Auto.getLayout("PathF", BuiltInLayouts.kList);
            public NetworkTableEntry PathFAngle1 = PathF.addPersistent("PathEAngle1", 0).getEntry();
            public NetworkTableEntry PathFDistence2 = PathF.addPersistent("PathEDistence2", 0).getEntry();
            public NetworkTableEntry PathFSpeed2 = PathF.addPersistent("PathESpeed2", 0).getEntry();

        private ShuffleboardLayout PathG = Auto.getLayout("PathG", BuiltInLayouts.kList);
            public NetworkTableEntry PathGAngle1 = PathG.addPersistent("PathGAngle1", 0).getEntry();
            public NetworkTableEntry PathGDistence2 = PathG.addPersistent("PathGDistence2", 0).getEntry();
            public NetworkTableEntry PathGSpeed2 = PathG.addPersistent("PathGSpeed2", 0).getEntry();
            public NetworkTableEntry PathGAngle3 = PathG.addPersistent("PathGAngle3", 0).getEntry();
            public NetworkTableEntry PathGTime4 = PathG.addPersistent("PathGTime4", 0).getEntry();
            public NetworkTableEntry PathGSpeed4 = PathG.addPersistent("PathGSpeed4", 0).getEntry();
        
        private ShuffleboardLayout PathH = Auto.getLayout("PathH", BuiltInLayouts.kList);
            public NetworkTableEntry PathHAngle1 = PathH.addPersistent("PathHAngle1", 0).getEntry();
            public NetworkTableEntry PathHTime2 = PathH.addPersistent("PathHTime2", 0).getEntry();
            public NetworkTableEntry PathHSpeed2 = PathH.addPersistent("PathHSpeed2", 0).getEntry();
            public NetworkTableEntry PathHAngle3 = PathH.addPersistent("PathHAngle3", 0).getEntry();

    public ShuffleBoard() {

    }
}   
