// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ControllerRumbleCallback;
import frc.robot.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RumbleState;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LockXOnAprilTag extends Command {
  private final SwerveDriveSubsystem swerve;
  private DoubleSupplier translationX; // TODO - use this!
  private DoubleSupplier heading;
  private double targetX;
  private ControllerRumbleCallback controllerRumbleCallback;

  private double startingYOffset;

  private PIDController yPIDController = new PIDController(0.01, 0, 0);
  private PIDController headingPIDController = new PIDController(1, 0, 0);
  //private PIDController yPIDController = new PIDController(0.1, 0, 0);


  // I think we may care more about the heading here as opposed to 'Y'
  // Strafe on X and only adjust heading to be consistent w/ the specific tag we are looking at
  // ie
  // we are zeroed facing directly towards an id on the reef
  // we detect the tag we are looking at and adjust the desired heading based on that and the original
  // 0 heading


  // x is front and back
  // y is left and right

  double targetTagId;
  Rotation2d targetHeading;

  private static final boolean DEBUG = true;

  public LockXOnAprilTag(SwerveDriveSubsystem swerveDriveSubsystem, DoubleSupplier translationX, DoubleSupplier heading, ControllerRumbleCallback controllerRumbleCallback) {
    this.swerve = swerveDriveSubsystem;
    this.translationX = translationX;
    this.heading = heading;
    this.controllerRumbleCallback = controllerRumbleCallback;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    startingYOffset = swerve.getTargetYOffset();
    targetTagId = LimelightHelpers.getFiducialID("limelight");
    // TODO - should check for the default value of 0 which would not help us

    targetHeading = FieldConstants.Reef.reefMap.get((int)targetTagId).left().getRotation();
  }

  @Override
  public void execute() {

        // TODO - this is bad?
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      if (SmartDashboard.getBoolean("Target Left Reef", true)) {
        LimelightHelpers.setPipelineIndex("limelight", 2);
      } else {
        LimelightHelpers.setPipelineIndex("limelight", 3);
      }
    } else {
      if (SmartDashboard.getBoolean("Target Left Reef", true)) {
        LimelightHelpers.setPipelineIndex("limelight", 4);
      } else {
        LimelightHelpers.setPipelineIndex("limelight",5);
      }
    }
    ////////////

    double translateY = 0;
    double headingAdjustment = 0;

    if(swerve.hasTarget()){
      translateY = yPIDController.calculate(swerve.getTargetXOffset(), 0);
      headingAdjustment = headingPIDController.calculate(
                    swerve.getSwerve().getOdometryHeading().getRadians(),
                    targetHeading.getRadians());

      //controllerRumbleCallback.update(RumbleState.TARGET_LOCKED_ON);
    }else{
      if (DEBUG) {
        System.out.println("Warning: Swerve Aim: Lost Target!");
      }
      controllerRumbleCallback.update(RumbleState.TARGET_NONE);
    }
 
    swerve.getSwerve().drive(
      new Translation2d(
      translationX.getAsDouble() * swerve.getSwerve().getMaximumChassisVelocity() ,
      (translateY) * swerve.getSwerve().getMaximumChassisVelocity()
      ),
      -headingAdjustment,
false,
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