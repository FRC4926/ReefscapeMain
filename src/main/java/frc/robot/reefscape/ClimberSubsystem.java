package frc.robot.reefscape;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem  extends SubsystemBase {
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

        SoftwareLimitSwitchConfigs softLimitConf = new SoftwareLimitSwitchConfigs()
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0)
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ClimberConstants.climbThres);

        // SoftwareLimitSwitchConfigs softLimitConf = new SoftwareLimitSwitchConfigs()
        // .withReverseSoftLimitEnable(false)
        // // .withReverseSoftLimitThreshold(0)
        // .withForwardSoftLimitEnable(false);
        // // .withForwardSoftLimitThreshold(ClimberConstants.climbThres)

        

        climbMotor1.setPosition(0);
        climbMotor2.setPosition(0);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.climbForwardCurrentLimit);
        climbMotor1.getConfigurator().apply(currentLimitsConfigs);
        climbMotor2.getConfigurator().apply(currentLimitsConfigs);

        climbMotor1.setNeutralMode(NeutralModeValue.Brake);
        climbMotor2.setNeutralMode(NeutralModeValue.Brake);

        
        climbMotor1.getConfigurator().apply(softLimitConf);
        climbMotor2.getConfigurator().apply(softLimitConf);
    }

    public void setVoltage(double velo)
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
        // if (climbMotor1.getPosition().getValueAsDouble() >= ClimberConstants.climbThres)
        // {
        //     climbZero();
        // } else
        // {
            CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.climbForwardCurrentLimit);
            climbMotor1.getConfigurator().apply(currentLimitsConfigs);
            climbMotor2.getConfigurator().apply(currentLimitsConfigs);
    
            return runOnce(() -> setVoltage(ClimberConstants.climbVelocityForward));
        // }
    }

    // public Command climbForwardCommand()
    // {
    //     if (climbMotor1.getPosition().getValueAsDouble() >= 105)
    //     {
    //         climbZero();
    //     } else
    //     {
    //         climbForward();
    //     }
    //     return run(this::climbForward);
    // }

    public Command climbBack()
    {   
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.climbBackCurrentLimit);
        climbMotor1.getConfigurator().apply(currentLimitsConfigs);
        climbMotor2.getConfigurator().apply(currentLimitsConfigs);
        // if (getClimb1Position() < ClimberConstants.climbThres)
        //     climbZero();
        // else
            // setVelocity(ClimberConstants.climbVelocityBack);

        return runOnce(() -> setVoltage(ClimberConstants.climbVelocityBack));
    }

    public Command climbBackCommand()
    {
        return run(this::climbBack);
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

    
}
