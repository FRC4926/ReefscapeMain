package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ThroughboreEncoder {

    //double startPosition = getAbsolutePosition();
    // private int x = 0;
    // private AsynchronousInterrupt interruptA, interruptB;
    // private void myCallback() {
    //     SmartDashboard.putNumber("number X", x);
    //     x++;
    // }
    private final Encoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    public ThroughboreEncoder(DigitalSource quadA, DigitalSource quadB, DigitalSource pwm, double distancePerRotation) {
        relativeEncoder = new Encoder(quadA, quadB);
        relativeEncoder.setDistancePerPulse(distancePerRotation/2048.0);

        absoluteEncoder = new DutyCycleEncoder(pwm);
        // interruptA = new AsynchronousInterrupt(quadA, (t, u) -> myCallback());
        // interruptA.enable();
        // interruptB = new AsynchronousInterrupt(quadB, (t, u) -> myCallback());
        // interruptB.enable();
        
    }

    public double getRelativePosition() {
        return relativeEncoder.getDistance();
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.get();
    }

    // public double getRefinedPosition() {
    //     return (relativeEncoder.get() + startPosition);
    // }
}
