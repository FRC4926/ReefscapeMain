package frc.robot.reefscape;

import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends ReefscapeBaseSubsystem {
    public final TalonFX leftMotor  = new TalonFX(ElevatorConstants.leftMotorCanId);
    public final TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorCanId);

    // private final Mechanism2d mech = new Mechanism2d(10.0, 10.0);
    // private final MechanismRoot2d mechRoot = mech.getRoot("elevator", 0.0, 0.0);

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

        Slot1Configs slot1Conf = new Slot1Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(ElevatorConstants.motorkG)
            .withKP(0.4)
            .withKI(ElevatorConstants.motorPidConstants.kI)
            .withKD(ElevatorConstants.motorPidConstants.kD);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(30);
        leftMotor.getConfigurator().apply(currentLimitsConfigs);
        rightMotor.getConfigurator().apply(currentLimitsConfigs);


        leftMotor.getConfigurator().apply(slot0Conf);
        rightMotor.getConfigurator().apply(slot0Conf);
        leftMotor.getConfigurator().apply(slot1Conf);
        rightMotor.getConfigurator().apply(slot1Conf);

        SoftwareLimitSwitchConfigs softLimitConf = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(false)
            // .withReverseSoftLimitThreshold(motorRotationsFromInches(ElevatorConstants.minPositionInches))
            .withForwardSoftLimitEnable(false);
            // .withForwardSoftLimitThreshold(motorRotationsFromInches(ElevatorConstants.maxPositionInches));
        
        
        leftMotor.getConfigurator().apply(softLimitConf);
        rightMotor.getConfigurator().apply(softLimitConf);

        CurrentLimitsConfigs currentLimitConf = new CurrentLimitsConfigs().withSupplyCurrentLimit(ElevatorConstants.currentLimit);
        leftMotor.getConfigurator().apply(currentLimitConf);
        rightMotor.getConfigurator().apply(currentLimitConf);
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
        return ElevatorConstants.levelsInches[stateIdx];
    }
    @Override
    void setReferencePosition(double position) {
        // SmartDashboard.putNumber("my position", position);
        // double convert = position * ElevatorConstants.gearRatio / ElevatorConstants.conversionFactor;
        // SmartDashboard.putNumber("my convert position", convert);

        // if (position > 3)
        // {
        //     leftMotor.setControl(new PositionVoltage(position* ElevatorConstants.gearRatio / ElevatorConstants.conversionFactor).withSlot(0));
        //     rightMotor.setControl(new PositionVoltage(position* ElevatorConstants.gearRatio / ElevatorConstants.conversionFactor).withSlot(0));
        // } else {
        //     leftMotor.setControl(new PositionVoltage(position* ElevatorConstants.gearRatio / ElevatorConstants.conversionFactor).withSlot(1));
        //     rightMotor.setControl(new PositionVoltage(position* ElevatorConstants.gearRatio / ElevatorConstants.conversionFactor).withSlot(1));
        // }

        int slot = (position > 3) ? 0 : 1;

        setToBothMotors(motor -> motor.setControl(new PositionVoltage(motorRotationsFromInches(position)).withSlot(slot)));
    }
    @Override
    void setReferenceVelocity(double velocity) {
        setToBothMotors(motor -> motor.setControl(new VelocityVoltage(motorRotationsFromInches(velocity))));
    }
    @Override
    public double getPosition() {
        return inchesFromMotorRotations(averageMotors(motor -> motor.getPosition().getValueAsDouble()));
    }
    @Override
    public double getVelocity() {
        return inchesFromMotorRotations(averageMotors(motor -> motor.getVelocity().getValueAsDouble()));
    }
    @Override
    public double getCurrent() {
        return sumMotors(motor -> motor.getStatorCurrent().getValueAsDouble());
    }

    // Utility methods

    private void setToBothMotors(Consumer<TalonFX> fn) {
        fn.accept(leftMotor);
        fn.accept(rightMotor);
    }

    private double sumMotors(Function<TalonFX, Double> fn) {
        return fn.apply(leftMotor) + fn.apply(rightMotor);
    }
    private double averageMotors(Function<TalonFX, Double> fn) {
        return 0.5 * sumMotors(fn);
    }

    private double inchesFromMotorRotations(double rotations) {
        return rotations * ElevatorConstants.inchesPerMotorRotation;
    }
    private double motorRotationsFromInches(double inches) {
        return inches / ElevatorConstants.inchesPerMotorRotation;
    }
}
