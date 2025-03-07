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
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.ElevatorAlgaeBarge;
import frc.robot.commands.ElevatorAlgaeL3;
import frc.robot.commands.ElevatorCoralL1;
import frc.robot.commands.ElevatorCoralL2;
import frc.robot.commands.ElevatorCoralL3;
import frc.robot.commands.ElevatorCoralL4;
import frc.robot.commands.ElevatorCoralStorage;
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
      });
  private final CoralSubystem coralSubystem = new CoralSubystem();
  private final AlgaeCollectorSubsystem algaeCollectorSubsystem = new AlgaeCollectorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveDriveSubsystem.getSwerveDrive(),
      () -> driver.getLeftY() * -1,
      () -> driver.getLeftX() * -1)
      .withControllerRotationAxis(driver::getRightX)
      .deadband(0.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
      driver::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
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

    // swerveDriveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    swerveDriveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);

    driver.start().onTrue((Commands.runOnce(swerveDriveSubsystem::zeroGyro)));
    driver.rightBumper().whileTrue(new LockHeadingOnAprilTag(swerveDriveSubsystem,
        () -> MathUtil.applyDeadband(-driver.getLeftY(), 0.05),
        () -> MathUtil.applyDeadband(-driver.getLeftX(), 0.05),
        new ControllerRumbleCallback() {
          @Override
          public void update(RumbleState rumbleState) {
            driverRumbleState = rumbleState;
          }
        }));

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
    driver.b().onTrue(new InstantCommand(elevator::down, elevator))
        .onFalse(new InstantCommand(elevator::stop, elevator));

    /*
     * driver
     * .a()
     * .onTrue(new InstantCommand(coralSubystem::shoot, coralSubystem))
     * .onFalse(new InstantCommand(coralSubystem::stop, coralSubystem));
     * 
     * driver
     * .b()
     * .onTrue(new InstantCommand(coralSubystem::returnToFunnel, coralSubystem))
     * .onFalse(new InstantCommand(coralSubystem::stop, coralSubystem));
     */

    /*
     * driver.a().onTrue(new InstantCommand(wristSubsystem::testPositionControl,
     * wristSubsystem));
     * 
     * driver.x().onTrue(new InstantCommand(wristSubsystem::testPositionControl2,
     * wristSubsystem));
     * 
     * 
     * driver.y().onTrue(new InstantCommand(wristSubsystem::testPositionControl3,
     * wristSubsystem));
     */

    // driver.b().onTrue(new InstantCommand(wristSubsystem::stop, wristSubsystem));

  }

  public void configureOperatorController() {
    //////////  
    // Coral Shooter
    operatorController.coralCollector().score().onTrue(coralSubystem.shoot()).onFalse(coralSubystem.stop());
    operatorController.coralCollector().reverse().onTrue(coralSubystem.reverse()).onFalse(coralSubystem.stop());

    //////////  
    // Elevator
    
    //// Manual Controls
    operatorController.elevator().up().onTrue(elevator.up()).onFalse(elevator.stop());
    operatorController.elevator().down().onTrue(elevator.down()).onFalse(elevator.stop());
    
    //// Algae Delivery Setpoints
    Command elevatorAlgaeBarge = new ElevatorAlgaeBarge(elevator);
    operatorController.elevator().algaeBarge().onTrue(elevatorAlgaeBarge);
    // Algae L3
    Command elevatorAlgaeL3 = new ElevatorAlgaeL3(elevator);
    operatorController.elevator().algaeL3().onTrue(elevatorAlgaeL3);
    // Algae L2
    operatorController.elevator().algaeL2().onTrue(new PrintCommand("Elevator - Algae L2 (L)"));
    // Algae Processor
    operatorController.elevator().algaeProcessor().onTrue(new PrintCommand("Elevator - Algae Processor (L)"));
    
    //// Coral Delivery Setpoints
    // Coral L4
    Command elevatorCoralL4 = new ElevatorCoralL4(elevator);
    operatorController.elevator().coralL4().onTrue(elevatorCoralL4);
    // Coral L3
    Command elevatorCoralL3 = new ElevatorCoralL3(elevator);
    operatorController.elevator().coralL3().onTrue(elevatorCoralL3);
    // Coral L2
    Command elevatorCoralL2 = new ElevatorCoralL2(elevator);
    operatorController.elevator().coralL2().onTrue(elevatorCoralL2);
    // Coral L1
    Command elevatorCoralL1 = new ElevatorCoralL1(elevator);
    operatorController.elevator().coralL1().onTrue(elevatorCoralL1);
    
    //// Storage
    // TODO - this needs to be expanded to include the AlgaeCollector position, verify that CoralCollector is safe, etc
    Command elevatorStorage = new ElevatorCoralStorage(elevator);
    operatorController.elevator().storage().onTrue(elevatorStorage);

    //////////  
    // Wrist

    //// Manual Controls
    operatorController.wrist().rotateDown().onTrue(new InstantCommand(wristSubsystem::down, wristSubsystem))
        .onFalse(new InstantCommand(wristSubsystem::stop, wristSubsystem));
    operatorController.wrist().rotateUp().onTrue(new InstantCommand(wristSubsystem::up, wristSubsystem))
        .onFalse(new InstantCommand(wristSubsystem::stop, wristSubsystem));
      
    //// Algae Collector Setpoints
    // Storage
    operatorController.algaeCollector().storage().onTrue(new PrintCommand("Algae - Storage (L)"));
    // Reef
    Command algaeReefCommand = new InstantCommand(wristSubsystem::algaeReef, wristSubsystem);
    operatorController.algaeCollector().reef().onTrue(algaeReefCommand);
    // Ground
    operatorController.algaeCollector().ground().onTrue(new PrintCommand("Algae - Ground (L)"));

    //////////  
    // Algae Collector
    Command algaeCollect = new InstantCommand(algaeCollectorSubsystem::in, algaeCollectorSubsystem);
    operatorController.algaeCollector().collect().onTrue(algaeCollect)
        .onFalse(new InstantCommand(algaeCollectorSubsystem::stop, algaeCollectorSubsystem));
    Command algaeRelease = new InstantCommand(algaeCollectorSubsystem::out, algaeCollectorSubsystem);
    operatorController.algaeCollector().release().onTrue(algaeRelease)
        .onFalse(new InstantCommand(algaeCollectorSubsystem::stop, algaeCollectorSubsystem));
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
