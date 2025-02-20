package frc.robot.reefscape;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends ReefscapeBaseSubsystem {
    private final SparkMax motor = new SparkMax(IntakeConstants.motorId, MotorType.kBrushless);
    private final DigitalInput innerProximitySensor = new DigitalInput(IntakeConstants.innerProximitySensorChannel);
    private final DigitalInput outerProximitySensor = new DigitalInput(IntakeConstants.outerProximitySensorChannel);

    public IntakeSubsystem() {
        super(true, true);

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
    void setReferenceVelocity(double velocity) {
        motor.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
    }
    @Override
    double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public void intake() {
        setReferenceVelocity(IntakeConstants.intakeVelocity);
    }
    public void outtake() {
        setReferenceVelocity(IntakeConstants.intakeVelocity);
    }
    public Command intakeCommand() {
        return runOnce(this::intake);
    }
    public Command outtakeCommand() {
        return runOnce(this::outtake);
    }

    public boolean isCoralInInnerIntake() {
        return innerProximitySensor.get();
    }
    public boolean isCoralInOuterIntake() {
        return outerProximitySensor.get();
    }

    // Utility methods

    // private double inchesFromMotorRotations(double rotations) {
    //     return rotations * IntakeConstants.inchesPerMotorRotation;
    // }
    // private double motorRotationsFromInches(double inches) {
    //     return inches / IntakeConstants.inchesPerMotorRotation;
    // }
}
