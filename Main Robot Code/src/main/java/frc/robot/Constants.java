/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int PCMID = 37;
    public static final int controllerPort = 0;

    public static final int intakeTalonPort = 0,
              ballFlickerTalonPort = 1,
              beltTalonPort = 2,
              outputTalonPort = 3,
             winchTalonPort = 4, 
                    elevatorTalonPort = 5,

               leftPort1 = 6,
				leftPort2 = 7,
				leftPort3 = 8,
				rightPort1 = 9,
				rightPort2 = 10,
				rightPort3 = 11;

  public static final int lowerIntakeSolenoidPort = 0,
              liftIntakeSolenoidPort = 1,
              lowerBlockSolenoidPort = 2,
              lowerUnBlockSolenoidPort = 3,
              upperBlockSolenoidPort = 4,
              upperUnBlockSolenoidPort = 5,
             pushSolenoidPort = 6,
             retractSolenoidPort = 7;
}
