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
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase{
    public final SparkMax motor = new SparkMax(AlgaeIntakeConstants.motorId, MotorType.kBrushless);
    // private final DigitalInput outerProximitySensor = new DigitalInput(AlgaeIntakeConstants.outerProximitySensorChannel);

    public AlgaeIntakeSubsystem() {


        SparkMaxConfig intakeMotorConf = new SparkMaxConfig();
        intakeMotorConf.encoder.velocityConversionFactor(AlgaeIntakeConstants.inchesPerMotorRotation);
        intakeMotorConf.encoder.positionConversionFactor(AlgaeIntakeConstants.inchesPerMotorRotation);
        intakeMotorConf.smartCurrentLimit(AlgaeIntakeConstants.currentLimit);
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
        setReferenceVelocity(AlgaeIntakeConstants.intakeVelocity);
    }

    /**public Command autonIntakeCommand(double velocity) {
        return runOnce(() -> setReferenceVelocity(velocity))
            .andThen(
                Commands.waitUntil(() -> (isCoralInInnerIntake() == true))
            )
            .andThen(runOnce(() -> setReferenceVelocity(0)));
    }**/
    
    public void intake() {
        double velocity = 0;
        velocity = AlgaeIntakeConstants.intakeVelocity;
        setReferenceVelocity(velocity);
    }

    public void outtake() {
        setReferenceVelocity(AlgaeIntakeConstants.outtakeVelocity);
    }

    public void zero()
    {
        setReferenceVelocity(0);
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
    



    // Utility methods

    // private double inchesFromMotorRotations(double rotations) {
    //     return rotations * AlgaeIntakeConstants.inchesPerMotorRotation;
    // }
    // private double motorRotationsFromInches(double inches) {
    //     return inches / AlgaeIntakeConstants.inchesPerMotorRotation;
    // }
}
