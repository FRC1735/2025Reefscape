// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorSubystemConstants;
import frc.robot.commands.LockHeadingOnAprilTag;
import frc.robot.commands.LockXOnAprilTag;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.CoralSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Controllers
  final CommandXboxController driver = new CommandXboxController(0);
  RumbleState driverRumbleState = RumbleState.TARGET_NONE;
  double driverRumbleIntensity = 0;

  // Subsystems
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(
                              new File(Filesystem.getDeployDirectory(), "swerve-2025"),
                              new ControllerRumbleCallback() {
                                  @Override
                                  public void update(RumbleState rumbleState) {
                                    driverRumbleState = rumbleState;
                                  }
                                }
                              );
  private final CoralSubystem coralSubystem = new CoralSubystem();
  private final AlgaeCollectorSubsystem algaeCollectorSubsystem = new AlgaeCollectorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveDriveSubsystem.getSwerveDrive(),
                                                                () -> driver.getLeftY() * -1,
                                                                () -> driver.getLeftX() * -1)
                                                            .withControllerRotationAxis(driver::getRightX)
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
                                                                                             driver::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
    .allianceRelativeControl(false);

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    Command driveRobotOrientedAngularVelocity = swerveDriveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngle = swerveDriveSubsystem.driveFieldOriented(driveDirectAngle);

    //swerveDriveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    swerveDriveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);

    driver.start().onTrue((Commands.runOnce(swerveDriveSubsystem::zeroGyro)));
    driver.rightBumper().whileTrue(new LockHeadingOnAprilTag(swerveDriveSubsystem, 
      ()->MathUtil.applyDeadband(-driver.getLeftY(), 0.05),
      ()->MathUtil.applyDeadband(-driver.getLeftX(), 0.05),
      new ControllerRumbleCallback() {
        @Override
        public void update(RumbleState rumbleState) {
          driverRumbleState = rumbleState;
        }
      }
      ));

    driver.leftBumper().whileTrue(new LockXOnAprilTag(
      swerveDriveSubsystem, 
      () -> 0, 
      driver::getRightX, 
      new ControllerRumbleCallback() {
        @Override
        public void update(RumbleState rumbleState) {
          // TODO
        }
      }));

      driver.a().onTrue(new InstantCommand(elevator::up, elevator)).onFalse(new InstantCommand(elevator::stop, elevator));
      driver.b().onTrue(new InstantCommand(elevator::down, elevator)).onFalse(new InstantCommand(elevator::stop, elevator));


      /*
      driver
        .a()
        .onTrue(new InstantCommand(coralSubystem::shoot, coralSubystem))
        .onFalse(new InstantCommand(coralSubystem::stop, coralSubystem));

      driver
        .b()
        .onTrue(new InstantCommand(coralSubystem::returnToFunnel, coralSubystem))
        .onFalse(new InstantCommand(coralSubystem::stop, coralSubystem));
        */

        /*
        driver.a().onTrue(new InstantCommand(wristSubsystem::testPositionControl, wristSubsystem));

        driver.x().onTrue(new InstantCommand(wristSubsystem::testPositionControl2, wristSubsystem));

        driver.y().onTrue(new InstantCommand(wristSubsystem::testPositionControl3, wristSubsystem));
        */

       //driver.b().onTrue(new InstantCommand(wristSubsystem::stop, wristSubsystem));


    }

  public void setRumble(double val) {
    driver.setRumble(RumbleType.kRightRumble, 1);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void rumblePeriodic() {
    if (DriverStation.isTeleopEnabled()) {
      switch (driverRumbleState) {
        case TARGET_FOUND:
          if (driverRumbleIntensity > 0.5) {
            driverRumbleIntensity = 0.2;
          } else {
            driverRumbleIntensity += 0.01;
          }
          break;
            
        case TARGET_LOCKED_ON:
          driverRumbleIntensity = 1;
          break;
            
        case TARGET_NONE:
        default:
          driverRumbleIntensity = 0;
          break;
      } 
    } else {
      driverRumbleState = RumbleState.TARGET_NONE;
    }

    driver.setRumble(RumbleType.kBothRumble, driverRumbleIntensity);
  }
}
