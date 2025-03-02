package frc.robot.reefscape;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
//import frc.robot.subsystems.ThroughboreEncoder;

public class PivotSubsystem extends ReefscapeBaseSubsystem {
    private final TalonFX motor = new TalonFX(PivotConstants.motorId);

    
    // TODO use throughbore encoder for pivot!!!
    // private final ThroughboreEncoder pivotEncoder = new ThroughboreEncoder(
    //     new DigitalInput(PivotConstants.motorEncoderAChannel),
    //     new DigitalInput(PivotConstants.motorEncoderBChannel),
    //     new DigitalInput(PivotConstants.motorEncoderPWMChannel), 1);

    public PivotSubsystem() {
        super(false, true);

        Slot0Configs motorConf0 = new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKG(PivotConstants.motorkG)
            .withKP(PivotConstants.motorPidConstants.kP)
            .withKI(PivotConstants.motorPidConstants.kI)
            .withKD(PivotConstants.motorPidConstants.kD);

        Slot1Configs motorConf1 = new Slot1Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKG(PivotConstants.motorkG)
            .withKP(PivotConstants.motorPidConstants.kP)
            .withKI(PivotConstants.motorPidConstants.kI)
            .withKD(PivotConstants.motorPidConstants.kD);

        motor.getConfigurator().apply(motorConf0);
        motor.getConfigurator().apply(motorConf1);
        motor.setPosition(0);
        motor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        motor.getConfigurator().apply(
            new CurrentLimitsConfigs().withSupplyCurrentLimit(PivotConstants.currentLimit));
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
        return PivotConstants.anglesDegrees[stateIdx];
    }
    @Override
    void setReferencePosition(double position) {
    
        int slot = (position > 3) ? 0 : 1;
        motor.setControl(new PositionVoltage(rotationsFromDegrees(position)).withSlot(slot));
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
        return rotations/32.0*360.0;
    }

    private double rotationsFromDegrees(double degrees)
    {
        return degrees*32.0/360.0;
    }

    // public void zeroPivot()
    // {
    //     motor.
    // }

    public double getConvertedPosition() {
        return motor.getPosition().getValueAsDouble()/32.0*360.0;
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
