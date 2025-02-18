// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControllerRumbleCallback;
import frc.robot.LimelightHelpers;
import frc.robot.RumbleState;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final boolean DEBUG = true;

  private final SwerveDrive swerveDrive;
  public double maximumSpeed = Units.feetToMeters(18.84);
  NetworkTableEntry validLimeLightTarget;
  NetworkTableEntry targetXOffset;
  NetworkTableEntry targetYOffset;
  private final ControllerRumbleCallback controllerRumbleCallback;

  public SwerveDriveSubsystem(File directory, ControllerRumbleCallback controllerRumbleCallback) {
    SwerveDriveTelemetry.verbosity = DEBUG ? TelemetryVerbosity.HIGH : TelemetryVerbosity.NONE;

    try {
      swerveDrive = new SwerveParser(directory)
                    .createSwerveDrive(maximumSpeed,
                                        new Pose2d(new Translation2d(Meter.of(1),
                                        Meter.of(4)),
                                        Rotation2d.fromDegrees(0)));
    } catch(Exception e) {
      throw new RuntimeException(e);
    }

    this.controllerRumbleCallback = controllerRumbleCallback;

    ////
    NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
    validLimeLightTarget = limeLightTable.getEntry("tv");
    targetXOffset = limeLightTable.getEntry("tx");
    targetYOffset = limeLightTable.getEntry("ty");
    /// 

    setupPathPlanner();
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void periodic() {

    if (hasTarget()) {
      controllerRumbleCallback.update(RumbleState.TARGET_FOUND);
    } else {
      controllerRumbleCallback.update(RumbleState.TARGET_NONE);
    }

    if (DEBUG) {
      SmartDashboard.putBoolean(
        "Has Target", hasTarget()
      );
      SmartDashboard.putNumber(
        "Target X Offset", getTargetXOffset()
      );
    }
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  private void setupPathPlanner() {
    // TODO
  }

  public boolean hasTarget() {    
    return validLimeLightTarget.getDouble(0.0) != 0.0;
  }

  public double getTargetXOffset() {
    return targetXOffset.getDouble(0.0);
  }

  public double getTargetYOffset() {
    return targetYOffset.getDouble(0.0);
  }

  public SwerveDrive getSwerve() {
    return swerveDrive;
  }
}
