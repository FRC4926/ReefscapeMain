package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CurrentTelemetry implements Sendable{
    public Supplier<double[]> currents;

    public CurrentTelemetry(Supplier<double[]> output) {
        this.currents = output;
    }
    public double sum() {
        double sum = 0;
        for (double current : currents.get()) {
            sum += current;
        }
        return sum;
    }
    public double avg() {
        return sum()/(currents.get().length);
    }
    // public double sumStator() {
    //     double sum = 0;
    //     for (double current : statorCurrents.get()) {
    //         sum += current;
    //     }
    //     return sum;
    // }
    // public double avgStator() {
    //     return sumStator()/(statorCurrents.get().length);
    // }
    // public double[] supplyStatorDiff() {
    //     double[] out = new double[supplyCurrents.get().length];
    //     for (int i = 0; i < supplyCurrents.get().length; i++) {
    //         out[i] = supplyCurrents.get()[i] - statorCurrents.get()[i];
    //     }
    //     return out;
    // }
    // public double supplyStatorDiffAvg() {
    //     double sum = 0;
    //     double len = 0;
    //     for (double current : supplyStatorDiff()) {
    //         sum += current;
    //         len++;
    //     }
    //     return sum/len;
    // }
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType(""); Do we need to use this?
        builder.addDoubleArrayProperty("Currents", currents, null);
        builder.addDoubleProperty("Sum", this::sum, null);
        builder.addDoubleProperty("Avg", this::avg, null);
        // builder.addDoubleArrayProperty("Currents: " + name + ": Stator", statorCurrents, null);
        // builder.addDoubleProperty("Currents: " + name + ": Stator Sum", this::sumStator, null);
        // builder.addDoubleProperty("Currents: " + name + ": Stator Avg", this::avgStator, null);
        // builder.addDoubleArrayProperty("Currents: " + name + ": Supply - Stator", this::supplyStatorDiff, null);
        // builder.addDoubleProperty("Currents: " + name + ": Supply - Stator Average", this::supplyStatorDiffAvg, null);
    }

}
