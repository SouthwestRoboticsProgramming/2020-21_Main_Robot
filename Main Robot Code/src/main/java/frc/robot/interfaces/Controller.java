// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

package frc.robot.interfaces;

public interface Controller {
     final Control<R<Doub>> ball;
     final Control<R<Doub>> climb;
     final Control<R<Doub>> drive;
     final Control<R<Doub>> wheel;
          
     default void rumble(double intensity, double duration) {
          new TimedCommand(duration) {
               void initalize() { rumble(intensity); }
               void end() { rumble(0); }
          }.start();
     }
     
     void rumble(double intensity);
     
     default void lights(int id, double duration); {
          new TimedCommand(duration) {
               void initalize() {  lights(id,true);    }
               void end() { lights(id,false);}
          }.start();
     }
     
     void lights(int id, boolean state);
     
     default public void init() {
          ball.whenActive(Robot.ballCommand);
          climb.whenActive(Robot.climbCommand);
          drive.whenActive(Robot.driveCommand);
          wheel.whenActive(Robot.wheelCommand):
     }
}

