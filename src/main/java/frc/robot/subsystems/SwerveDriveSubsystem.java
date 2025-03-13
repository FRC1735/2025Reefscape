// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(0.01, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.01, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
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
