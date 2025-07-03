package frc.robot.reefscape;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ReefscapeState;

public class Reefscape {
    private ReefscapeState currentState = ReefscapeState.Home;
    private ReefscapeState lastLevel = ReefscapeState.Level2;

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final PivotSubsystem pivot = new PivotSubsystem();
    public final AlgaeSubsytem algae = new AlgaeSubsytem();
    public final IntakeSubsystem intake = new IntakeSubsystem();

    public Reefscape() {
    }

    // Applies `state` to elevator, pivot, algae, and intake
    public void applyState(ReefscapeState state) {
        applyState(state, true, true, true);
    }

    // Applies the current state of the subsystem to the elevator, pivot, and/or
    // intake
    public void applyState(boolean applyToElevator, boolean applyToPivot, boolean applyToAlgae) {
        applyState(currentState, applyToElevator, applyToPivot, applyToAlgae);
    }

    public void applyState(ReefscapeState state, boolean applyToElevator, boolean applyToPivot, boolean applyToAlgae) {

    }

    public Command applyStateCommand(ReefscapeState state) {
        return applyStateCommand(state, true, true, true);
    }

    public Command applyStateCommand(Supplier<ReefscapeState> stateSupplier) {
        return applyStateCommand(stateSupplier, true, true, true);
    }

    public Command applyStateCommand(ReefscapeState state, boolean applyToElevator, boolean applyToPivot, boolean applyToAlgae) {
        ParallelCommandGroup command = new ParallelCommandGroup();
        command.addCommands(new InstantCommand(() -> {
            currentState = state;
            if (state.isLevel())
                lastLevel = state;
        }));
        if (applyToElevator)
            command.addCommands(elevator.setStateCommand(state));
        if (applyToPivot)
            command.addCommands(pivot.setStateCommand(state));
        if(applyToAlgae)
            command.addCommands(algae.setStateCommand(state));

        return command;
    }

    public Command applyStateCommand(Supplier<ReefscapeState> stateSupplier, boolean applyToElevator, boolean applyToPivot, boolean applyToAlgae) {
        ParallelCommandGroup command = new ParallelCommandGroup();
        command.addCommands(new InstantCommand(() -> {
            ReefscapeState state = stateSupplier.get();
            currentState = state;
            if (state.isLevel())
                lastLevel = state;
        }));
        if (applyToElevator)
            command.addCommands(elevator.setStateCommand(stateSupplier));
        if (applyToPivot)
            command.addCommands(pivot.setStateCommand(stateSupplier));

        return command;
    }

    public Command applyStateCommandManual(Supplier<ReefscapeState> stateSupplier, boolean applyToElevator, boolean applyToPivot, boolean applyToAlgae) {
        ParallelCommandGroup command = new ParallelCommandGroup();
        command.addCommands(new InstantCommand(() -> {
            ReefscapeState state = stateSupplier.get();
            currentState = state;
        }));
        if (applyToElevator)
            command.addCommands(elevator.setStateCommand(stateSupplier));
        if (applyToPivot)
            command.addCommands(pivot.setStateCommand(stateSupplier));
 
        return command;
    }

    public Command applyStateCommandManual(ReefscapeState state, boolean applyToElevator, boolean applyToPivot, boolean applyToAlgae) {
        ParallelCommandGroup command = new ParallelCommandGroup();
        command.addCommands(new InstantCommand(() -> currentState = state));
        if (applyToElevator)
            command.addCommands(elevator.setStateCommand(state));
        if (applyToPivot)
            command.addCommands(pivot.setStateCommand(state));

        return command;
    }

    public Command applyStateCommand(boolean applyToElevator, boolean applyToPivot, boolean applyToAlgae, boolean applyToIntake) {
        return applyStateCommand(currentState, applyToElevator, applyToPivot, applyToAlgae);
    }

    public ReefscapeState getState() {
        return currentState;
    }

    public ReefscapeState getElevatorState() {
        return elevator.getState();
    }

    public ReefscapeState getPivotState() {
        return pivot.getState();
    }

    public ReefscapeState getAlgaeState() {
        return algae.getState();
    }

    public ReefscapeState getLastLevel() {
        return lastLevel;
    }

    public boolean isCoralInInnerIntake() {
        return intake.isCoralInInnerIntake();
    }

    public void toggleElevatorManual() {
        elevator.toggleManualControl();
    }

    public Command toggleElevatorManualCommand() {
        return elevator.toggleManualControlCommand();
    }

    public Trigger elevatorIsManual() {
        return new Trigger(() -> elevator.isManualControl());
    }

    public Command elevatorMoveWithVelocityCommand(double velocity) {
        return elevator.setManualVelocityCommand(() -> velocity);
    }

    public Command elevatorMoveWithVelocityCommand(DoubleSupplier velocity) {
        return elevator.setManualVelocityCommand(velocity);
    }

    public Command levelCommand() {
        return intake.levelCommand();
    }

    public Command autonLevelCommand() {
        return intake.autonLevelCommand();
    }

    public Command intakeCommand() {
        return intake.intakeCommand();
    }

    public Command outtakeCommand() {
        return intake.outtakeCommand();
    }

    public Command zeroCommand() {
        return intake.zeroIntake();
    }
}
