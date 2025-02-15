package frc.robot.reefscape;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ReefscapeState;

public abstract class ReefscapeBaseSubsystem implements Subsystem {
    protected final SysIdRoutine sysIdRoutine;
    protected ReefscapeState currentState = ReefscapeState.Home;
    protected ReefscapeState backupState = null;
    protected boolean velocityBased;

    abstract void setVoltage(Voltage voltage);
    abstract Voltage getVoltage();
    abstract double getParameterFromArray(int stateIdx);
    abstract void setReferencePosition(double position);
    abstract void setReferenceVelocity(double velocity);
    abstract double getPosition();
    abstract double getVelocity();
    // abstract double getCurrentPosition();
    // abstract double getCurrentVelocity();
    // abstract double getCurrentVoltage();

    public ReefscapeBaseSubsystem(boolean _velocityBased) {
        velocityBased = _velocityBased;

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Units.Volts.of(4.0),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> setVoltage(voltage), null, this
            ));
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public boolean isManualControl() {
        return currentState == ReefscapeState.Manual;
    }
    public void setManualControl(boolean manual) {
        if (manual == isManualControl())
            return;
            
        if (manual == true) {
            backupState = currentState;
            currentState = ReefscapeState.Manual;
        } else {
            currentState = backupState;
        }
    }
    public void toggleManualControl() {
        setManualControl(!isManualControl());
    }
    public Command toggleManualControlCommand() {
        return runOnce(() -> toggleManualControl());
    }

    public void setManualPosition(double position) {
        if (!isManualControl())
            return;
        
        setReferencePosition(position);
    }
    public Command setManualPositionCommand(DoubleSupplier positionSupplier) {
        return run(() -> setManualPosition(positionSupplier.getAsDouble()));
    }
    public void setManualVelocity(double velocity) {
        if (!isManualControl())
            return;
        
        setReferenceVelocity(velocity);
    }
    public Command setManualVelocityCommand(DoubleSupplier velocitySupplier) {
        return run(() -> setManualVelocity(velocitySupplier.getAsDouble()));
    }

    public void setState(ReefscapeState state) {
        if (!isManualControl())
            return;
        
        double parameter = getParameterFromArray(state.ordinal());
        if (velocityBased)
            setReferenceVelocity(parameter);
        else
            setReferencePosition(parameter);
    }
    public Command setStateCommand(ReefscapeState state) {
        return runOnce(() -> setState(state));
    }
    public Command setStateCommand(Supplier<ReefscapeState> stateSupplier) {
        return runOnce(() -> setState(stateSupplier.get()));
    }
    public ReefscapeState getState() {
        return currentState;
    }
}
