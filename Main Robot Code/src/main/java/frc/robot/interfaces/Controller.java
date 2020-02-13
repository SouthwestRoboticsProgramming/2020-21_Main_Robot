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
     void rumble(double x);
     void lights(int id, boolean state);
}

