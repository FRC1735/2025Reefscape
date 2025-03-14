// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ControllerRumbleCallback;
import frc.robot.LimelightHelpers;
import frc.robot.RumbleState;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LockHeadingOnAprilTag extends Command {
  private final SwerveDriveSubsystem swerve;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private double heading;
  private double lastGoodHeading;
  private ControllerRumbleCallback controllerRumbleCallback;

  private static final boolean DEBUG = false;

  public LockHeadingOnAprilTag(SwerveDriveSubsystem swerveDriveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY, ControllerRumbleCallback controllerRumbleCallback) {
    this.swerve = swerveDriveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    lastGoodHeading = 0;
    this.controllerRumbleCallback = controllerRumbleCallback;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {

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


    if(swerve.hasTarget()){
      // TODO - adjustements assuming we are targeting the pole to the left of the tag

      // TODO - need ability to switch left or right on targeting (-7 or +7)

      boolean targetLeft = true;

      heading = (
        -(swerve.getTargetXOffset() 
        //+ (targetLeft? 7 : -7) 
        //+ ((targetLeft? -0.1: 0.1) * LimelightHelpers.getTY("limelight"))
        ) / 70);
      controllerRumbleCallback.update(RumbleState.TARGET_LOCKED_ON);
    }else{
      if (DEBUG) {
        System.out.println("Warning: Swerve Aim: Lost Target!");
      }
      controllerRumbleCallback.update(RumbleState.TARGET_NONE);
      heading = 0;
    }
    lastGoodHeading = heading;
 
    swerve.getSwerve().drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerve.getSwerve().getMaximumChassisVelocity(),
    Math.pow(translationY.getAsDouble(), 3) * swerve.getSwerve().getMaximumChassisVelocity()),
    heading * swerve.getSwerve().getMaximumChassisAngularVelocity(),
true,
false);

    if (DEBUG) {
      SmartDashboard.putNumber(
        "Last good heading", lastGoodHeading
      );
      SmartDashboard.putNumber(
        "Translation X", translationX.getAsDouble()
      );
      SmartDashboard.putNumber(
        "Translation Y", translationY.getAsDouble()
      );

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