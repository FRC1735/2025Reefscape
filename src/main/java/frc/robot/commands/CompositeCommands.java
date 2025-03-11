// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.KeyboardController.CoralCollector;
import frc.robot.utils.KeyboardController.Wrist;

/** Add your docs here. */
public class CompositeCommands {

    // General
    public static Command elevatorStorage(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.storage(),
            wrist.algaeStorage()
        );
    }

    // Coral Specific
    public static Command elevatorCoralL1(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.coralL1(),
            wrist.algaeStorage()
        );
    }

    public static Command elevatorCoralL2(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.coralL2(),
            wrist.algaeStorage()
        );
    }

    public static Command elevatorCoralL3(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.coralL3(),
            wrist.algaeStorage()
        );
    }

    public static Command elevatorCoralL4(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.coralL4(),
            wrist.algaeStorage()
        );
    }

    // Algae specific
    public static Command elevatorAlgaeProcessor(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.algaeProcessor(),
            wrist.algaeProcessor()
        );
    }

    public static Command elevatorAlgaeL2(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.algaeL2(),
            wrist.algaeL2()
        );
    }

    public static Command elevatorAlgaeL3(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.algaeL3(),
            wrist.algaeL3()
        );
    }

    public static Command elevatorAlgaeBarge(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new ParallelCommandGroup(
            elevator.algaeBarge(),
            wrist.algaeBarge()
        );
    }

}
