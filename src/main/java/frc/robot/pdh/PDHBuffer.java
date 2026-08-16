package frc.robot.pdh;

public final class PDHBuffer {
    public double voltage_V = Double.NaN;
    public double totalCurrent_A = Double.NaN;
    public double[] currents_A = new double[0];
    public boolean[] breakersTripped = new boolean[0];
    public boolean connected = false;
}
