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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorSubystemConstants;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.EleveatorAlgaeL3;
import frc.robot.commands.LockHeadingOnAprilTag;
import frc.robot.commands.LockXOnAprilTag;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.KeyboardController;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Controllers
  final CommandXboxController driver = new CommandXboxController(0);
  RumbleState driverRumbleState = RumbleState.TARGET_NONE;
  double driverRumbleIntensity = 0;
  final KeyboardController operatorController = new KeyboardController(0);

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
  private final ClimberSubsystem climber = new ClimberSubsystem();


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
    configureDriverController();
    configureOperatorController();
  }

  public void configureDriverController() {
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

  public void configureOperatorController() {
    operatorController.test().test().onTrue(new PrintCommand("KEYPAD SAYS HI!"));

    // TODO - theres gotta be a way to merge these

    // score with coral
    operatorController.coralCollector().score_L().onTrue(new PrintCommand("Coral - Score (L)"));
    operatorController.coralCollector().score_R().onTrue(new PrintCommand("Coral - Score (R)"));

    // reverse coral
    operatorController.coralCollector().reverse_L().onTrue(new PrintCommand("Coral - Reverse (L)"));
    operatorController.coralCollector().reverse_R().onTrue(new PrintCommand("Coral - Reverse (R)"));

    // climber up
    operatorController.climber().up_L().onTrue(new PrintCommand("Climber - Up (L)"));
    operatorController.climber().up_R().onTrue(new PrintCommand("Climber - Up (R)"));

    // climber down
    operatorController.climber().down_L().onTrue(new PrintCommand("Climber - Down (L)"));
    operatorController.climber().down_R().onTrue(new PrintCommand("Climber - Down (R)"));

    // wrist rotation
    operatorController.wrist().rotateDown().onTrue(new PrintCommand("Wrist - Rotate Down"));
    operatorController.wrist().rotateUp().onTrue(new PrintCommand("Wrist - Rotate Up"));

    // algae storage
    operatorController.algaeCollector().storage_L().onTrue(new PrintCommand("Algae - Storage (L)"));
    operatorController.algaeCollector().storage_R().onTrue(new PrintCommand("Algae - Storage (R)"));

    // algae reef
    operatorController.algaeCollector().reef_L().onTrue(new PrintCommand("Algae - Reef (L)"));
    operatorController.algaeCollector().reef_R().onTrue(new PrintCommand("Algae - Reef (R)"));

    // algae ground
    operatorController.algaeCollector().ground_L().onTrue(new PrintCommand("Algae - Ground (L)"));
    operatorController.algaeCollector().ground_R().onTrue(new PrintCommand("Algae - Ground (R)"));

    // algae collect
    operatorController.algaeCollector().collect_L().onTrue(new PrintCommand("Algae - Collect (L)"));
    operatorController.algaeCollector().collect_R().onTrue(new PrintCommand("Algae - Collect (R)"));

    // algae release
    operatorController.algaeCollector().release_L().onTrue(new PrintCommand("Algae - Release (L)"));
    operatorController.algaeCollector().release_R().onTrue(new PrintCommand("Algae - Release (R)"));

    // elevator manual control - up
    Command elevatorUp = new ElevatorUp(elevator);
    operatorController.elevator().up_L().whileTrue(elevatorUp);
    operatorController.elevator().up_R().whileTrue(elevatorUp);

    // elevator manual control - down
    Command elevatorDown = new ElevatorDown(elevator);
    operatorController.elevator().down_L().whileTrue(elevatorDown);
    operatorController.elevator().down_R().whileTrue(elevatorDown);

    // elevator positions (compound commands across subsystems I assume)

    // Algae Barge
    operatorController.elevator().algaeBarge_L().onTrue(new PrintCommand("Elevator - Algae Barge (L)"));
    operatorController.elevator().algaeBarge_R().onTrue(new PrintCommand("Elevator - Algae Barge (R)"));

    // Algae L3
    Command elevatorAlgaeL3 = new EleveatorAlgaeL3(elevator);
    operatorController.elevator().algaeL3_L().onTrue(elevatorAlgaeL3);
    operatorController.elevator().algaeL3_R().onTrue(elevatorAlgaeL3);

    // Algae L2
    operatorController.elevator().algaeL2_L().onTrue(new PrintCommand("Elevator - Algae L2 (L)"));
    operatorController.elevator().algaeL2_R().onTrue(new PrintCommand("Elevator - Algae L2 (R)"));

    // Algae Processor
    operatorController.elevator().algaeProcessor_L().onTrue(new PrintCommand("Elevator - Algae Processor (L)"));
    operatorController.elevator().algaeProcessor_R().onTrue(new PrintCommand("Elevator - Algae Processor (R))"));

    // Coral L4
    operatorController.elevator().coralL4_L().onTrue(new PrintCommand("Elevator - Coral L4 (L)"));
    operatorController.elevator().coralL4_R().onTrue(new PrintCommand("Elevator - Coral L4 (R)"));

    // Coral L3
    operatorController.elevator().coralL3_L().onTrue(new PrintCommand("Elevator - Coral L3 (L)"));
    operatorController.elevator().coralL3_R().onTrue(new PrintCommand("Elevator - Coral L3 (R)"));

    // Coral L2
    operatorController.elevator().coralL2_L().onTrue(new PrintCommand("Elevator - Coral L2 (L)"));
    operatorController.elevator().coralL2_R().onTrue(new PrintCommand("Elevator - Coral L2 (R)"));

    // Coral L1
    operatorController.elevator().coralL1_L().onTrue(new PrintCommand("Elevator - Coral L1 (L)"));
    operatorController.elevator().coralL1_R().onTrue(new PrintCommand("Elevator - Coral L1 (R)"));

    // Storage
    operatorController.elevator().storage_A().onTrue(new PrintCommand("Storage (A)"));
    operatorController.elevator().storage_B().onTrue(new PrintCommand("Storage (B)"));
    operatorController.elevator().storage_C().onTrue(new PrintCommand("Storage (C)"));
    operatorController.elevator().storage_D().onTrue(new PrintCommand("Storage (D)"));
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
