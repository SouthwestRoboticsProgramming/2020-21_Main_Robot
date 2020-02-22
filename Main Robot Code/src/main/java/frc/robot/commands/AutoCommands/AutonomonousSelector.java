/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AutoCommands.AutoPaths.AutoSequence;
import frc.robot.commands.AutoCommands.AutoPaths.PathC;
import frc.robot.commands.AutoCommands.AutoPaths.PathD;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutonomonousSelector extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private BallSubsystem ballSubsystem;

  public AutonomonousSelector(DriveTrainSubsystem driveTrainSubsystem, BallSubsystem ballSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.ballSubsystem = ballSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AutonomonousSelector.initialize()");
    new AutoSequence(driveTrainSubsystem, ballSubsystem).schedule();
    // new PathC(driveTrainSubsystem, ballSubsystem).schedule();
    // new PathD(driveTrainSubsystem, ballSubsystem).schedule();
    // new FollowWithLimelight(driveTrainSubsystem, 5, .15).schedule();
    // new DriveTime(driveTrainSubsystem, 2, .25).schedule();;
    // new PathH(driveTrainSubsystem, ballSubsystem).schedule();
    // new setBallMode(ballSubsystem, ballMode.intake);
    // new AccelerateDrive(driveTrainSubsystem, 0, -.25, Wheel.both);
    // new PathBNew(driveTrainSubsystem, ballSubsystem).schedule();
    // runPath(Robot.shuffleBoard.path1.getString(""));
    // runPath(Robot.shuffleBoard.path2.getString(""));
    // runPath(Robot.shuffleBoard.path3.getString(""));
    // runPath(Robot.shuffleBoard.path4.getString(""));
    // new GetAcclDecelDistence(driveTrainSubsystem).schedule();
    // new TurnToAngle(driveTrainSubsystem, 30, Wheel.both).schedule();;
    // new DriveDistence(driveTrainSubsystem, 5, .5).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  private void runPath(String path) {
    if (path.toUpperCase() == "A") {
      // new PathA(driveTrainSubsystem).schedule();;
    } else if (path.toUpperCase() == "B") {
      // new PathB(driveTrainSubsystem).schedule();
    } else {

    }
  }
}
