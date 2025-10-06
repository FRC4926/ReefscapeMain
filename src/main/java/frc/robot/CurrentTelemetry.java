package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CurrentTelemetry implements Sendable{
    public Supplier<double[]> supplyCurrents;
    public Supplier<double[]> statorCurrents;
    public String name;
    public CurrentTelemetry(Supplier<double[]> stator, Supplier<double[]> supply, String name) {
        this.supplyCurrents = supply;
        this.statorCurrents = stator;
        this.name = name;
    }
    public double sumSupply() {
        double sum = 0;
        for (double current : supplyCurrents.get()) {
            sum += current;
        }
        return sum;
    }
    public double avgSupply() {
        return sumSupply()/(supplyCurrents.get().length);
    }
    public double sumStator() {
        double sum = 0;
        for (double current : statorCurrents.get()) {
            sum += current;
        }
        return sum;
    }
    public double avgStator() {
        return sumStator()/(statorCurrents.get().length);
    }
    public double[] supplyStatorDiff() {
        double[] out = new double[supplyCurrents.get().length];
        for (int i = 0; i < supplyCurrents.get().length; i++) {
            out[i] = supplyCurrents.get()[i] - statorCurrents.get()[i];
        }
        return out;
    }
    public double supplyStatorDiffAvg() {
        double sum = 0;
        double len = 0;
        for (double current : supplyStatorDiff()) {
            sum += current;
            len++;
        }
        return sum/len;
    }
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty("Currents: " + name + ": Supply", supplyCurrents, null);
        builder.addDoubleProperty("Currents: " + name + ": Supply Sum", this::sumSupply, null);
        builder.addDoubleProperty("Currents: " + name + ": Supply Avg", this::avgSupply, null);
        builder.addDoubleArrayProperty("Currents: " + name + ": Stator", statorCurrents, null);
        builder.addDoubleProperty("Currents: " + name + ": Stator Sum", this::sumStator, null);
        builder.addDoubleProperty("Currents: " + name + ": Stator Avg", this::avgStator, null);
        builder.addDoubleArrayProperty("Currents: " + name + ": Supply - Stator", this::supplyStatorDiff, null);
        builder.addDoubleProperty("Currents: " + name + ": Supply - Stator Average", this::supplyStatorDiffAvg, null);
    }

}
