// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.CoralSubystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // Subsystems
  //private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve-2025"));
  //private final CoralSubystem coralSubystem = new CoralSubystem();
  //private final AlgaeCollectorSubsystem algaeCollectorSubsystem = new AlgaeCollectorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  
 // Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  /*
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveDriveSubsystem.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
                                                            */

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  /*
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);
*/

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  /*
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
    .allianceRelativeControl(false);
    */

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /*
    Command driveRobotOrientedAngularVelocity = swerveDriveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngle = swerveDriveSubsystem.driveFieldOriented(driveDirectAngle);

    //swerveDriveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    swerveDriveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);

    driverXbox.start().onTrue((Commands.runOnce(swerveDriveSubsystem::zeroGyro)));
    */

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
