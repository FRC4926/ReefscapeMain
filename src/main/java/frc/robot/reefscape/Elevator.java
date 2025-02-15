package frc.robot.reefscape;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ReefscapeState;

public class Elevator {
    private final TalonFX leftMotor  = new TalonFX(ElevatorConstants.leftMotorCanId);
    private final TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorCanId);
    
    private ReefscapeState currentState;

    public Elevator(ReefscapeState initialState) {
        currentState = initialState;
        
        Slot0Configs conf = new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(ElevatorConstants.motorkG)
            .withKP(ElevatorConstants.motorPidConstants.kP)
            .withKI(ElevatorConstants.motorPidConstants.kI)
            .withKD(ElevatorConstants.motorPidConstants.kD);

        leftMotor.getConfigurator().apply(conf);
        rightMotor.getConfigurator().apply(conf);
    }

    public void moveToPosition(double position) {
        if (!isManual())
            return;
        leftMotor.setControl(new PositionVoltage(ElevatorConstants.gearRatio * position));
        rightMotor.setControl(new PositionVoltage(ElevatorConstants.gearRatio * position));
    }

    public void moveWithVelocity(double velocity) {
        if (!isManual())
            return;
        leftMotor.setControl(new VelocityVoltage(ElevatorConstants.gearRatio * velocity));
        rightMotor.setControl(new VelocityVoltage(ElevatorConstants.gearRatio * velocity));
    }

    public void moveToLevel(ReefscapeState state) {
        currentState = state;
        double height = ElevatorConstants.levels[state.ordinal()];
        leftMotor.setControl(new PositionVoltage(ElevatorConstants.gearRatio * height));
        rightMotor.setControl(new PositionVoltage(ElevatorConstants.gearRatio * height));
    }

    public void setManual() {
        currentState = null;
    }
    public boolean isManual() {
        return currentState == null;
    }

    public double getPosition() {
        return 0.5*(leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / ElevatorConstants.gearRatio;
    }
    public ReefscapeState getState() {
        return currentState;
    }

    public void moveRelatively(double offset) {
        moveToPosition(getPosition() + offset);
    }
}
