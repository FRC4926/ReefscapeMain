package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ReefscapeState;
import frc.robot.reefscape.Elevator;
import frc.robot.reefscape.Intake;
import frc.robot.reefscape.Pivot;

public class ReefscapeSubsystem implements Subsystem {
    private ReefscapeState currentState = ReefscapeState.Home;

    private final Elevator elevator = new Elevator(currentState);
    private final Pivot pivot = new Pivot(currentState);
    private final Intake intake = new Intake(currentState);

    public ReefscapeSubsystem() {
    }

    // Applies `state` to elevator, pivot, and intake
    public void applyState(ReefscapeState state) {
        applyState(state, true, true, true);
    }
    // Applies the current state of the subsystem to the elevator, pivot, and/or intake
    public void applyState(boolean applyToElevator, boolean applyToPivot, boolean applyToIntake) {
        applyState(currentState, applyToElevator, applyToPivot, applyToIntake);
    }
    public void applyState(ReefscapeState state, boolean applyToElevator, boolean applyToPivot, boolean applyToIntake) {
        currentState = state;
        if (applyToElevator)
            elevator.moveToLevel(state);
        if (applyToPivot)
            pivot.pivotTo(state);
        if (applyToIntake)
            intake.setState(state);
    }
    public Command applyStateCommand(ReefscapeState state) {
        return applyStateCommand(state, true, true, true);
    }
    public Command applyStateCommand(ReefscapeState state, boolean applyToElevator, boolean applyToPivot, boolean applyToIntake) {
        return runOnce(() -> applyState(state, applyToElevator, applyToPivot, applyToIntake));
    }
    public Command applyStateCommand(Supplier<ReefscapeState> stateSupplier, boolean applyToElevator, boolean applyToPivot, boolean applyToIntake) {
        return runOnce(() -> applyState(stateSupplier.get(), applyToElevator, applyToPivot, applyToIntake));
    }
    public Command applyStateCommand(Supplier<ReefscapeState> stateSupplier) {
        return applyStateCommand(stateSupplier, true, true, true);
    }
    public ReefscapeState getState() {
        return currentState;
    }
    public ReefscapeState getElevatorState() {
        return elevator.getState();
    }
    public ReefscapeState getPivotState() {
        return pivot.getCurrentState();
    }
    public ReefscapeState getIntakeState() {
        return intake.getState();
    }

    public boolean coralInIntake() {
        return intake.coralInIntake();
    }
}
