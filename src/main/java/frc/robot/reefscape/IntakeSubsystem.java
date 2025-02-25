package frc.robot.reefscape;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ReefscapeState;

public class IntakeSubsystem extends ReefscapeBaseSubsystem {
    public final SparkMax motor = new SparkMax(IntakeConstants.motorId, MotorType.kBrushless);
    public final AnalogInput innerProximitySensor = new AnalogInput(IntakeConstants.innerProximitySensorChannel);
    // private final DigitalInput outerProximitySensor = new DigitalInput(IntakeConstants.outerProximitySensorChannel);

    public IntakeSubsystem() {
        super(true, true);

        innerProximitySensor.setAverageBits(4);

        SparkMaxConfig intakeMotorConf = new SparkMaxConfig();
        // TODO does this work???
        intakeMotorConf.encoder.velocityConversionFactor(IntakeConstants.inchesPerMotorRotation);
        intakeMotorConf.encoder.positionConversionFactor(IntakeConstants.inchesPerMotorRotation);
        intakeMotorConf.closedLoop
            .pid(
                IntakeConstants.motorPidConstants.kP,
                IntakeConstants.motorPidConstants.kI,
                IntakeConstants.motorPidConstants.kD
            );
        intakeMotorConf.smartCurrentLimit(IntakeConstants.currentLimit);
        motor.configure(intakeMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }
    @Override
    Voltage getVoltage() {
        return Voltage.ofRelativeUnits(motor.getBusVoltage(), Units.Volts);
    }
    @Override
    double getParameterFromArray(int stateIdx) {
        return IntakeConstants.velocitiesInchesPerSecond[stateIdx];
    }
    @Override
    void setReferencePosition(double position) {
        motor.getClosedLoopController().setReference(position, ControlType.kPosition);
    }
    @Override
    double getPosition() {
        return motor.getEncoder().getPosition();
    }
    @Override
    void setReferenceVelocity(double effort) {

        motor.set(effort);
    }
    @Override
    double getVelocity() {
        return motor.getEncoder().getVelocity();
    }
    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    public void level()
    {
        setReferenceVelocity(IntakeConstants.lowerIntakeVelocity);
    }
    public void intake() {
        ReefscapeState current = getState();
        double velocity = 0;

       if (isCoralInInnerIntake())
        {
            velocity = 0;
        } else {
            velocity = IntakeConstants.intakeVelocity;
        }
        // }
        // if (current == ReefscapeState.Home)
        // {
        //     velocity = 0;
        // } else if (current == ReefscapeState.CoralStation && isCoralInInnerIntake())
        // {
        //     velocity = 0;

        // } else if (current == ReefscapeState.CoralStation && !isCoralInInnerIntake())
        // {
        //     velocity = IntakeConstants.intakeVelocity;
        // } else if (current.isLevel())
        // {
        //     velocity = IntakeConstants.intakeVelocity;
        // }
        setReferenceVelocity(velocity);
        //(IntakeConstants.intakeVelocity);
    }
    public void outtake() {
        setReferenceVelocity(IntakeConstants.outtakeVelocity);
    }

    public void zero()
    {
        setReferenceVelocity(0);
    }

    public Command levelCommand()
    {
        return runOnce(this::level);
    }
    public Command intakeCommand() {
        return run(this::intake);
    }
    public Command outtakeCommand() {
        return runOnce(this::outtake);
    }

    public  Command zeroIntake()
    {
        return runOnce(this::zero);
    }
    

    public boolean isCoralInInnerIntake() {
        return innerProximitySensor.getAverageVoltage() > 2.0;
    }
    public boolean isCoralInOuterIntake() {
       return motor.getOutputCurrent() > 25.0;
    }


    // Utility methods

    // private double inchesFromMotorRotations(double rotations) {
    //     return rotations * IntakeConstants.inchesPerMotorRotation;
    // }
    // private double motorRotationsFromInches(double inches) {
    //     return inches / IntakeConstants.inchesPerMotorRotation;
    // }
}
