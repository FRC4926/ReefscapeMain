package frc.robot.reefscape;

import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class ClimberSubsystem extends SubsystemBase {
    public final TalonFX climbMotor1  = new TalonFX(ClimberConstants.climb1Id);
    public final TalonFX climbMotor2 = new TalonFX(ClimberConstants.climb2Id);

    private final DutyCycleOut climb1Out = new DutyCycleOut(0);
    private final DutyCycleOut climb2Out = new DutyCycleOut(0);

    // private final Mechanism2d mech = new Mechanism2d(10.0, 10.0);
    // private final MechanismRoot2d mechRoot = mech.getRoot("elevator", 0.0, 0.0);

    public ClimberSubsystem() {
  

        // climbMotor1.getConfigurator().apply(
        //     new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        // );
        // climbMotor2.getConfigurator().apply(
        //     new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
        // );

        

        climbMotor1.setPosition(0);
        climbMotor2.setPosition(0);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.climbForwardCurrentLimit);
        climbMotor1.getConfigurator().apply(currentLimitsConfigs);
        climbMotor2.getConfigurator().apply(currentLimitsConfigs);

        climbMotor1.setNeutralMode(NeutralModeValue.Brake);
        climbMotor2.setNeutralMode(NeutralModeValue.Brake);


        SoftwareLimitSwitchConfigs softLimitConf = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(false)
            // .withReverseSoftLimitThreshold(motorRotationsFromInches(ElevatorConstants.minPositionInches))
            .withForwardSoftLimitEnable(false);
            // .withForwardSoftLimitThreshold(motorRotationsFromInches(ElevatorConstants.maxPositionInches));
        
        
        climbMotor1.getConfigurator().apply(softLimitConf);
        climbMotor2.getConfigurator().apply(softLimitConf);
    }

    public void setVelocity(double velo)
    {
        climb1Out.Output = velo;
        climb2Out.Output = velo;
        climbMotor1.setControl(climb1Out);
        climbMotor2.setControl(climb2Out);
    }

    public void zeroClimb()
    {
        // climb1Out.Output = 0;
        // climb2Out.Output = 0;
        climbMotor1.setControl(new VelocityVoltage(0));
        climbMotor2.setControl(new VelocityVoltage(0));
    }

    public double getClimb1Position()
    {
        return climbMotor1.getPosition().getValueAsDouble();
    }

    public double getClimb2Position()
    {
        return climbMotor2.getPosition().getValueAsDouble();
    }

    public double getClimb1Converted()
    {
        return getClimb1Position()/ClimberConstants.climberRatio;
    }
    
    public double getClimb2Converted()
    {
        return getClimb1Position()/ClimberConstants.climberRatio;
    }

    public double getClimbPosition()
    {
        return 0.5 * (getClimb1Position() + getClimb2Position());
    }

    public Command climbForward()
    {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.climbForwardCurrentLimit);
        climbMotor1.getConfigurator().apply(currentLimitsConfigs);
        climbMotor2.getConfigurator().apply(currentLimitsConfigs);

        return runOnce(() -> setVelocity(ClimberConstants.climbVelocityForward));
    }

    public Command climbBack()
    {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.climbBackCurrentLimit);
        climbMotor1.getConfigurator().apply(currentLimitsConfigs);
        climbMotor2.getConfigurator().apply(currentLimitsConfigs);

        return runOnce(() -> setVelocity(ClimberConstants.climbVelocityBack));
    }
    
    public Command climbZero()
    {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.climbBackCurrentLimit);
        climbMotor1.getConfigurator().apply(currentLimitsConfigs);
        climbMotor2.getConfigurator().apply(currentLimitsConfigs);

        return runOnce(() -> zeroClimb());
    }

    public double getCurrent1()
    {
        return climbMotor1.getStatorCurrent().getValueAsDouble();
    }

    public Command zeroClimbCommand(double set)
    {
        return runOnce(() -> zeroClimb());
    }

    public double getCurrent() {
        return climbMotor1.getStatorCurrent().getValueAsDouble() + climbMotor2.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {

    }

    // Utility methods

    private void setToBothMotors(Consumer<TalonFX> fn) {
        fn.accept(climbMotor1);
        fn.accept(climbMotor2);
    }
    private double sumMotors(Function<TalonFX, Double> fn) {
        return fn.apply(climbMotor1) + fn.apply(climbMotor2);
    }
    private double averageMotors(Function<TalonFX, Double> fn) {
        return 0.5 * sumMotors(fn);
    }

}
