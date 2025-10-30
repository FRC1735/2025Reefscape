// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackAprilTagCommand extends Command {
  SwerveDriveSubsystem driveline;

  /** Creates a new TrackAprilTagCommand. */
  public TrackAprilTagCommand(SwerveDriveSubsystem driveline) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveline);

    this.driveline = driveline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //driveline.getSwerve().drive(null, 0, isFinished(), isScheduled());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(driveline.getTargetXOffset());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
