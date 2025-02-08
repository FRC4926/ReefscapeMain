package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem implements Subsystem {
    private final TalonFX leftMotor  = new TalonFX(ElevatorConstants.leftMotorCanId);
    private final TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorCanId);
    

    private boolean manualControl = false;

    public ElevatorSubsystem() {
        Slot0Configs conf = new Slot0Configs();
        conf.GravityType = GravityTypeValue.Elevator_Static;
        conf.kG = ElevatorConstants.motorkG;
        conf.kP = ElevatorConstants.motorPidConstants.kP;
        conf.kI = ElevatorConstants.motorPidConstants.kI;
        conf.kD = ElevatorConstants.motorPidConstants.kD;

        leftMotor.getConfigurator().apply(conf);
        rightMotor.getConfigurator().apply(conf);
    }

    public void moveToPosition(double position) {
        leftMotor.setControl(new PositionVoltage(ElevatorConstants.gearRatio * position));
        rightMotor.setControl(new PositionVoltage(ElevatorConstants.gearRatio * position));
    }

    public void moveWithVelocity(double velocity) {
        if (!manualControl)
            return;
        leftMotor.setControl(new VelocityVoltage(ElevatorConstants.gearRatio * velocity));
        rightMotor.setControl(new VelocityVoltage(ElevatorConstants.gearRatio * velocity));
    }

    public Command moveWithVelocityCommand(DoubleSupplier input) {
        return runOnce(() -> moveWithVelocity(input.getAsDouble()));
    }

    public void moveToLevel(int level) {
        // if ((level > ElevatorConstants.levels.length - 1) || (ElevatorConstants.levels[level] == -1))
        //     return;
        if (manualControl)
            return;
        leftMotor.setControl(new PositionVoltage(ElevatorConstants.levels[level]));
        rightMotor.setControl(new PositionVoltage(ElevatorConstants.levels[level]));
    }

    public Command moveToLevelCommand(int level) {
        if ((level > ElevatorConstants.levels.length - 1) || ElevatorConstants.levels[level] == -1)
            return null;
        return runOnce(() -> moveToLevel(level));
    }

    public void toggleManual() {
        manualControl = !manualControl;
    }

    public Command toggleManualCommand() {
        return runOnce(() -> toggleManual());
    }

    public double getPosition() {
        return 0.5*(leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / ElevatorConstants.gearRatio;
    }

    public void moveRelatively(double offset) {
        moveToPosition(getPosition() + offset);
    }
}
