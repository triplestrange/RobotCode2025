package com.team1533.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.team1533.frc2025.subsystems.elevator.Elevator;

public class ElevatorCommands {
    public static Command moveToPosition(Elevator elevator, double height) {
        return new InstantCommand(() -> elevator.setTargetPosition(height), elevator);
    }

    public static Command liftThenDrop(Elevator elevator, double liftHeight) {
        return new SequentialCommandGroup(
            moveToPosition(elevator, liftHeight),
            moveToPosition(elevator, 0.0) // Drop 
        );
    }
}
