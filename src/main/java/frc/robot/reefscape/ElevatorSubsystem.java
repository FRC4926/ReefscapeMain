package frc.robot.reefscape;

import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends ReefscapeBaseSubsystem {
    public final TalonFX leftMotor  = new TalonFX(ElevatorConstants.leftMotorCanId);
    public final TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorCanId);

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

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(30);
        leftMotor.getConfigurator().apply(currentLimitsConfigs);
        rightMotor.getConfigurator().apply(currentLimitsConfigs);


        leftMotor.getConfigurator().apply(slot0Conf);
        rightMotor.getConfigurator().apply(slot0Conf);

        SoftwareLimitSwitchConfigs softLimitConf = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(false)
            // .withReverseSoftLimitThreshold(ElevatorConstants.minPositionInches / ElevatorConstants.conversionFactor)
            .withForwardSoftLimitEnable(false);
            // .withForwardSoftLimitThreshold(ElevatorConstants.maxPositionInches / ElevatorConstants.conversionFactor);
        
        
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
    public void periodic() {

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
        SmartDashboard.putNumber("my position", position);
        double convert = position * ElevatorConstants.gearRatio / ElevatorConstants.conversionFactor;
        SmartDashboard.putNumber("my convert position", convert);

        leftMotor.setControl(new PositionVoltage(position* ElevatorConstants.gearRatio / ElevatorConstants.conversionFactor));
        rightMotor.setControl(new PositionVoltage(position* ElevatorConstants.gearRatio / ElevatorConstants.conversionFactor));
        // setToBothMotors(motor -> motor.setControl(new PositionVoltage(position / (ElevatorConstants.gearRatio * ElevatorConstants.conversionFactor))));
    }
    @Override
    void setReferenceVelocity(double velocity) {
        setToBothMotors(motor -> motor.setControl(new VelocityVoltage(ElevatorConstants.conversionFactor * velocity / ElevatorConstants.gearRatio*ElevatorConstants.conversionFactor)));
    }
    @Override
    public double getPosition() {
        return averageMotors(motor -> ElevatorConstants.conversionFactor*motor.getPosition().getValueAsDouble() /ElevatorConstants.gearRatio);
    }
    @Override
    double getVelocity() {
        return averageMotors(motor -> motor.getVelocity().getValueAsDouble()) * ElevatorConstants.gearRatio*ElevatorConstants.conversionFactor;
    }
}
