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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    public final SparkMax motor = new SparkMax(IntakeConstants.motorId, MotorType.kBrushless);
    public final AnalogInput innerProximitySensor = new AnalogInput(IntakeConstants.innerProximitySensorChannel);
    // private final DigitalInput outerProximitySensor = new DigitalInput(IntakeConstants.outerProximitySensorChannel);

    public IntakeSubsystem() {
        innerProximitySensor.setAverageBits(4);

        SparkMaxConfig intakeMotorConf = new SparkMaxConfig();
        intakeMotorConf.encoder.velocityConversionFactor(IntakeConstants.inchesPerMotorRotation);
        intakeMotorConf.encoder.positionConversionFactor(IntakeConstants.inchesPerMotorRotation);
        intakeMotorConf.smartCurrentLimit(IntakeConstants.currentLimit);
        motor.configure(intakeMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    Voltage getVoltage() {
        return Voltage.ofRelativeUnits(motor.getBusVoltage(), Units.Volts);
    }

    double getPosition() {
        return motor.getEncoder().getPosition();
    }

    void setReferenceVelocity(double effort) {

        motor.set(effort);
    }

    double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    public void level()
    {
        setReferenceVelocity(IntakeConstants.intakeVelocity);
    }

    public Command autonIntakeCommand(double velocity) {
        return runOnce(() -> setReferenceVelocity(velocity))
            .andThen(
                Commands.waitUntil(() -> (isCoralInInnerIntake() == true))
            )
            .andThen(runOnce(() -> setReferenceVelocity(0)));
    }
    
    public void intake() {
        double velocity = 0;

       if (isCoralInInnerIntake())
        {
            velocity = 0;
        } else {
            velocity = IntakeConstants.intakeVelocity;
        }
        setReferenceVelocity(velocity);
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

    public Command autonLevelCommand()
    {
        return run(this::level);
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
        return innerProximitySensor.getAverageVoltage() > IntakeConstants.cliffSensorThreshold;
    }



    // Utility methods

    // private double inchesFromMotorRotations(double rotations) {
    //     return rotations * IntakeConstants.inchesPerMotorRotation;
    // }
    // private double motorRotationsFromInches(double inches) {
    //     return inches / IntakeConstants.inchesPerMotorRotation;
    // }
}
