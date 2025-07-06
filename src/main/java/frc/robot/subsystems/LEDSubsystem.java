package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(LEDConstants.port);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.numLeds);
    private int x = 0;
    private int y=0;
    private int counter = 0;
    LEDPattern scrollingGalactechRainbow = (reader, writer) -> {
        SmartDashboard.putNumber("len", reader.getLength()); 
        for (int i = 0; i < LEDConstants.numLeds; i++) {
            if (0 <= (i - x) && (i - x) <= 2) {
                writer.setLED(i, Color.kGreen);
            } else {
                writer.setLED(i, Color.kPurple);
            }
        }
        x++;
        x %= LEDConstants.numLeds;
    };

    double b = 0;

    
    LEDPattern hasCoralIn = (reader, writer) -> {
        for (int i = 0; i < LEDConstants.numLeds; i++) {
             writer.setLED(i, Color.kWhite);
          }
    };
    LEDPattern hasNoCoral = (reader, writer) -> {
        for (int i = 0; i < LEDConstants.numLeds; i++) {
             writer.setLED(i, Color.kRed);
          }
    };
    LEDPattern seesTag = (reader, writer) -> {
        for (int i = 0; i < LEDConstants.numLeds; i++) {
             writer.setLED(i, Color.kGreen);
          }
    };
    LEDPattern inAlign = (reader, writer) -> {
        for (int i = 0; i < LEDConstants.numLeds; i++) {
            if ((Math.round(counter/10))%2==0)
                writer.setLED(i, Color.kBlack);
            else
                writer.setLED(i, Color.kGreen);
          }
    };
        
    

    LEDPattern scrollingGalactechRainbow2 = (reader, writer) -> {
        SmartDashboard.putNumber("len", reader.getLength());
        for (int i = 0; i < LEDConstants.numLeds; i++) {
            if(i<=1){
                writer.setLED(i, Color.kGreen);
            }
            else if (i<=x-2) {
                writer.setLED(i, Color.kGreen);  
            } else {
                writer.setLED(i, Color.kPurple);
            }
        }

        //b is for bias, so it wants to go up when at the bottom and wants to go down at the top

        b = 0;
        if(x > 13){
            b = 0.5;
        }
        if(x < 6){
            b = -0.5;
        }

        if(y%3==0){
        x = x + (int)((Math.round(Math.random()) - (0.5 + b)) * 2);
        if(x < 2){
            x = 2;
        }
        if(x > 15){
            x = 15;
        }
        y=0;
        }
    };

    public void setLEDState(BooleanSupplier hasCoral, BooleanSupplier canAlign, BooleanSupplier isAligning)
    {
        if(hasCoral.getAsBoolean()){
            if (canAlign.getAsBoolean())
            {
                if (isAligning.getAsBoolean())
                {
                    inAlign.applyTo(ledBuffer);
                    led.setData(ledBuffer);
                } else
                {
                    seesTag.applyTo(ledBuffer);
                    led.setData(ledBuffer);
                }
            } else{
                hasCoralIn.applyTo(ledBuffer);
                led.setData(ledBuffer);
            }
        }
        else {
            hasNoCoral.applyTo(ledBuffer);
            led.setData(ledBuffer);
        }
    }


    public LEDSubsystem() {

        led.setLength(LEDConstants.numLeds);
        led.setData(ledBuffer);
        led.start();

        
    }

    @Override
    public void periodic() {
        counter++;
    }



    // public void setColor(Color color) {
    //     LEDPattern.solid(color).applyTo(ledBuffer);
    //     led.setData(ledBuffer);
    // }
    // public void doThing() {
    //     LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kPurple)
    //         .applyTo(ledBuffer);
    // }
    // public Command runRainbow() {
    //     LEDPattern rainbow = LEDPattern.rainbow(255, 100)
    //         .scrollAtAbsoluteSpeed(InchesPerSecond.of(10.0), Inches.of(1.0));
        
    //     return run(() -> rainbow.applyTo(ledBuffer));
    // }
}
