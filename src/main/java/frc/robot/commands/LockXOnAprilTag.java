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

  private double startingYOffset;

  private PIDController xPIDController = new PIDController(0.01, 0, 0);
  private PIDController yPIDController = new PIDController(0.1, 0, 0);

  // I think we may care more about the heading here as opposed to 'Y'
  // Strafe on X and only adjust heading to be consistent w/ the specific tag we are looking at
  // ie
  // we are zeroed facing directly towards an id on the reef
  // we detect the tag we are looking at and adjust the desired heading based on that and the original
  // 0 heading

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
    startingYOffset = swerve.getTargetYOffset();
  }

  @Override
  public void execute() {
    double translateX = 0;
    double translateY = 0;

    if(swerve.hasTarget()){
      translateX = xPIDController.calculate(swerve.getTargetXOffset(), 0);
      translateY = yPIDController.calculate(swerve.getTargetYOffset(), startingYOffset);

      //controllerRumbleCallback.update(RumbleState.TARGET_LOCKED_ON);
    }else{
      if (DEBUG) {
        System.out.println("Warning: Swerve Aim: Lost Target!");
      }
      controllerRumbleCallback.update(RumbleState.TARGET_NONE);
    }
 
    swerve.getSwerve().drive(
      new Translation2d(-translateY * swerve.getSwerve().getMaximumChassisVelocity(),
      (-translateX) * swerve.getSwerve().getMaximumChassisVelocity()
      ),
    heading.getAsDouble() * swerve.getSwerve().getMaximumChassisAngularVelocity(),
true,
false);

    if (DEBUG) {
      SmartDashboard.putNumber("Target Y Offset", startingYOffset);
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