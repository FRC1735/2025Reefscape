// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackAprilTagCommand extends Command {
  SwerveDriveSubsystem driveline;
  PIDController apirlTagController = new PIDController(.1, 0, 0);

  /** Creates a new TrackAprilTagCommand. */
  public TrackAprilTagCommand(SwerveDriveSubsystem driveline) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveline);

    this.driveline = driveline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOffset = (driveline.getTargetXOffset());
    /* 
    if (Math.abs(xOffset) < 6){
      stop();
      turn(0);
    }
     else if (xOffset > 0) {
      goRight();
      turn(.5);
    }else{
      goLeft();
      turn(-.5);
    }*/

    turn(-apirlTagController.calculate(xOffset, 0));
  }

  private void turn(double speed) {
    driveline.getSwerve().drive(
      new Translation2d(
        0,
        0
      ),
      speed,
      false,
      false);
  }

  private void goRight() {
    driveline.getSwerve().drive(
      new Translation2d(
        0,
        -1 * driveline.getSwerve().getMaximumChassisVelocity()
      ),
      0,
      false,
      false);

  }
  private void goLeft() {
    driveline.getSwerve().drive(
      new Translation2d(
        0,
        1 * driveline.getSwerve().getMaximumChassisVelocity()
      ),
      0,
      false,
      false);
  }
  private void stop() {
    
      driveline.getSwerve().drive(
        new Translation2d(
          0,
          0
        ),
        0,
        false,
        false);
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
