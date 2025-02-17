package frc.robot.reefscape;

import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends ReefscapeBaseSubsystem {
    private final TalonFX leftMotor  = new TalonFX(ElevatorConstants.leftMotorCanId);
    private final TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorCanId);

    public ElevatorSubsystem() {
        super(false, false);

        leftMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );
        rightMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
        );

        Slot0Configs slot0Conf = new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(ElevatorConstants.motorkG)
            .withKP(ElevatorConstants.motorPidConstants.kP)
            .withKI(ElevatorConstants.motorPidConstants.kI)
            .withKD(ElevatorConstants.motorPidConstants.kD);

        leftMotor.getConfigurator().apply(slot0Conf);
        rightMotor.getConfigurator().apply(slot0Conf);

        SoftwareLimitSwitchConfigs softLimitConf = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ElevatorConstants.minPositionMeters / ElevatorConstants.metersPerRotation)
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ElevatorConstants.maxPositionMeters / ElevatorConstants.metersPerRotation);
        
        // CurrentLimitsConfigs currentLimitConf = new CurrentLimitsConfigs()
        //     .
        
        leftMotor.getConfigurator().apply(softLimitConf);
        rightMotor.getConfigurator().apply(softLimitConf);
    }

    private void setToBothMotors(Consumer<TalonFX> fn) {
        fn.accept(leftMotor);
        fn.accept(rightMotor);
    }
    private double averageMotors(Function<TalonFX, Double> fn) {
        return 0.5*(fn.apply(leftMotor) + fn.apply(rightMotor));
    }

    @Override
    void setVoltage(Voltage voltage) {
        setToBothMotors((motor) -> motor.setControl(new VoltageOut(voltage)));
    }
    @Override
    Voltage getVoltage() {
        return Voltage.ofRelativeUnits(averageMotors((motor) -> motor.getMotorVoltage().getValue().in(Units.Volts)), Units.Volts);
    }
    @Override
    double getParameterFromArray(int stateIdx) {
        return ElevatorConstants.levels[stateIdx];
    }
    @Override
    void setReferencePosition(double position) {
        setToBothMotors(motor -> motor.setControl(new PositionVoltage(position / ElevatorConstants.metersPerRotation)));
    }
    @Override
    void setReferenceVelocity(double velocity) {
        setToBothMotors(motor -> motor.setControl(new VelocityVoltage(velocity / ElevatorConstants.metersPerRotation)));
    }
    @Override
    double getPosition() {
        return averageMotors(motor -> motor.getPosition().getValueAsDouble()) * ElevatorConstants.metersPerRotation;
    }
    @Override
    double getVelocity() {
        return averageMotors(motor -> motor.getVelocity().getValueAsDouble()) * ElevatorConstants.metersPerRotation;
    }
}
