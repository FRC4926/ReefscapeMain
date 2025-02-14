package frc.robot.reefscape;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ReefscapeState;

public class Intake {
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.motorId, MotorType.kBrushless);
    private final DigitalInput proximitySensor = new DigitalInput(IntakeConstants.proximitySensorChannel);

    private ReefscapeState currentState;

    public Intake(ReefscapeState initialState) {
        currentState = initialState;
        
        SparkMaxConfig intakeMotorConf = new SparkMaxConfig();
        intakeMotorConf.encoder.velocityConversionFactor(IntakeConstants.metersPerRotation);
        intakeMotorConf.closedLoop
            .pid(
                IntakeConstants.motorPidConstants.kP,
                IntakeConstants.motorPidConstants.kI,
                IntakeConstants.motorPidConstants.kD
            );
        intakeMotor.configure(intakeMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean coralInIntake() {
        return proximitySensor.get();
    }


    // Positive = intake, negative = outtake
    public void setSpeed(double speedMetersPerSecond) {                   
        currentState = null;
        intakeMotor.getClosedLoopController().setReference(speedMetersPerSecond, ControlType.kVelocity);
    }
    public void setState(ReefscapeState state) {
        currentState = state;
        double metersPerSecond = IntakeConstants.velocitiesMetersPerSecond[state.ordinal()];
        intakeMotor.getClosedLoopController().setReference(metersPerSecond, ControlType.kVelocity);
    }
    public ReefscapeState getState() {
        return currentState;
    }
    public double getSpeedMetersPerSecond() {
        return intakeMotor.getEncoder().getVelocity();
    }
}
