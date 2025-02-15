// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ControllerRumbleCallback;
import frc.robot.RumbleState;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LockXOnAprilTag extends Command {
  private final SwerveDriveSubsystem swerve;
  private DoubleSupplier translationY; // TODO - use this!
  private DoubleSupplier heading;
  private double targetX;
  private ControllerRumbleCallback controllerRumbleCallback;

  private PIDController xPIDController = new PIDController(0.01, 0, 0);

  private static final boolean DEBUG = true;

  public LockXOnAprilTag(SwerveDriveSubsystem swerveDriveSubsystem, DoubleSupplier translationY, DoubleSupplier heading, ControllerRumbleCallback controllerRumbleCallback) {
    this.swerve = swerveDriveSubsystem;
    this.translationY = translationY;
    this.heading = heading;
    this.controllerRumbleCallback = controllerRumbleCallback;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double translateX = 0;
    if(swerve.hasTarget()){
      translateX = xPIDController.calculate(swerve.getTargetXOffset(), 0);
      //controllerRumbleCallback.update(RumbleState.TARGET_LOCKED_ON);
    }else{
      if (DEBUG) {
        System.out.println("Warning: Swerve Aim: Lost Target!");
      }
      controllerRumbleCallback.update(RumbleState.TARGET_NONE);
    }
 
    swerve.getSwerve().drive(
      new Translation2d(translationY.getAsDouble() * swerve.getSwerve().getMaximumChassisVelocity(),
      (-translateX) * swerve.getSwerve().getMaximumChassisVelocity()
      ),
    heading.getAsDouble() * swerve.getSwerve().getMaximumChassisAngularVelocity(),
true,
false);

    if (DEBUG) {

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerRumbleCallback.update(RumbleState.TARGET_NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}