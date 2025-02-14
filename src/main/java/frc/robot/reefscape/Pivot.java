package frc.robot.reefscape;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ReefscapeState;
import frc.robot.subsystems.ThroughboreEncoder;

// TODO integrate pivotEncoder with the rest of the Pivot
public class Pivot {
    private final TalonFX pivotMotor   = new TalonFX(PivotConstants.motorId);
    private final ThroughboreEncoder pivotEncoder = new ThroughboreEncoder(
        new DigitalInput(PivotConstants.motorEncoderAChannel),
        new DigitalInput(PivotConstants.motorEncoderBChannel),
        new DigitalInput(PivotConstants.motorEncoderPWMChannel), 1);

    private ReefscapeState currentState;

    public Pivot(ReefscapeState initialState) {
        currentState = initialState;

        Slot0Configs motorConf = new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(PivotConstants.motorkG)
            .withKP(PivotConstants.motorPidConstants.kP)
            .withKI(PivotConstants.motorPidConstants.kI)
            .withKD(PivotConstants.motorPidConstants.kD);
        pivotMotor.getConfigurator().apply(motorConf);
    }

    public ReefscapeState getCurrentState() {
        return currentState;
    }

    public void pivotTo(ReefscapeState state) {
        double degrees = PivotConstants.anglesDegrees[state.ordinal()];
        pivotMotor.setControl(new PositionVoltage(degrees / 360.0));
    }
    public void pivotTo(Rotation2d angle) {
        currentState = null;
        pivotMotor.setControl(new PositionVoltage(angle.getRotations()));
    }

    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(pivotMotor.getPosition().getValueAsDouble());
    }
}
