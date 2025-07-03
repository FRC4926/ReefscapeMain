package frc.robot.reefscape;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsytem extends ReefscapeBaseSubsystem {
    private final TalonFX motor = new TalonFX(AlgaeConstants.motorId);

    public AlgaeSubsytem() {
        super(false, true);

        Slot0Configs motorConf0 = new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKG(AlgaeConstants.motorkG)
            .withKP(AlgaeConstants.motorPidConstants.kP)
            .withKI(AlgaeConstants.motorPidConstants.kI)
            .withKD(AlgaeConstants.motorPidConstants.kD);

        motor.setNeutralMode(NeutralModeValue.Brake);


        motor.getConfigurator().apply(motorConf0);
        motor.setPosition(0);
        motor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        motor.getConfigurator().apply(
            new CurrentLimitsConfigs().withSupplyCurrentLimit(AlgaeConstants.currentLimit));
    }

    @Override
    void setVoltage(Voltage voltage) {
        motor.setControl(new VoltageOut(voltage));
    }
    @Override
    Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }
    @Override
    double getParameterFromArray(int stateIdx) {
        return AlgaeConstants.anglesDegrees[stateIdx];
    }
    @Override
    void setReferencePosition(double position) {
        motor.setControl(new PositionVoltage(rotationsFromDegrees(position)));
    }
    @Override
    void setReferenceVelocity(double velocity) {
        motor.setControl(new VelocityVoltage(velocity));
    }
    @Override
    public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    private double degreesFromMotorRotations(double rotations) {
        return rotations/AlgaeConstants.gearRatio*360.0;
    }

    private double rotationsFromDegrees(double degrees)
    {
        return degrees*AlgaeConstants.gearRatio/360.0;
    }

    public void zeroAlgae()
    {
        motor.setControl(new VelocityVoltage(0.0));
    }

    public double getConvertedPosition() {
        return degreesFromMotorRotations(motor.getPosition().getValueAsDouble());
    }
    
    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }
    @Override
    double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }
}
