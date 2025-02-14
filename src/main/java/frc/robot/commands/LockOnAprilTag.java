// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ControllerRumbleCallback;
import frc.robot.RumbleState;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LockOnAprilTag extends Command {
  private final SwerveDriveSubsystem swerve;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private double heading;
  private double lastGoodHeading;
  private ControllerRumbleCallback controllerRumbleCallback;

  private static final boolean DEBUG = true;

  /** Creates a new driveAimAtTarget. */
  public LockOnAprilTag(SwerveDriveSubsystem swerveDriveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY, ControllerRumbleCallback controllerRumbleCallback) {
    this.swerve = swerveDriveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    lastGoodHeading = 0;
    this.controllerRumbleCallback = controllerRumbleCallback;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO - For now I only care about locking onto any april tag in view, could 
    // TODO - add something like this to improve.
    //swerve.setVisionTargetID(7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(swerve.hasTarget()){
      //heading = Math.pow(-SwerveSub.getVisionAngle()/30,3);
      heading = -swerve.getTargetXOffset() / 70;
      
      // TODO - would be cool to utilize haptic feedback to notify driver that this is working
      //obotContainer.setRightRumbleDriver(0);
      controllerRumbleCallback.update(RumbleState.TARGET_LOCKED_ON);
    }else{
      System.out.println("Warning: Swerve Aim: Lost Target!");
      // TODO - would be cool to utilize haptic feedback to notify driver that this is working
      //RobotContainer.setRightRumbleDriver(1);
      controllerRumbleCallback.update(RumbleState.TARGET_NONE);
      heading = 0;
    }
    lastGoodHeading = heading;

    //System.out.println("Target is: " + heading);
    //SwerveSub.driveCommand(translationX, translationY, heading);

    swerve.getSwerve().drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerve.getSwerve().getMaximumChassisVelocity(),
    Math.pow(translationY.getAsDouble(), 3) * swerve.getSwerve().getMaximumChassisVelocity()),
    heading * swerve.getSwerve().getMaximumChassisAngularVelocity(),
true,
false);

    if (DEBUG) {
      SmartDashboard.putNumber(
        "Last good heading", lastGoodHeading
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.setRightRumbleDriver(0);
    //System.out.println("Warning: Swerve Aim: Target Lost!");

    controllerRumbleCallback.update(RumbleState.TARGET_NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return !SwerveSub.isValidVisionTarget();
    return false;
  }
}