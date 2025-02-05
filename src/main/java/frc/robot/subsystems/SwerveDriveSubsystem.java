// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final boolean DEBUG = true;

  private final SwerveDrive swerveDrive;
  public double maximumSpeed = Units.feetToMeters(18.84);

  public SwerveDriveSubsystem(File directory) {
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
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  private void setupPathPlanner() {
    // TODO
  }
}
