package frc.robot.pdh;

public final class PDHBuffer {
    public double voltage_V = 0;
    public double totalCurrent_A = 0;
    public double[] currents_A = new double[0];
    public boolean[] breakersTripped = new boolean[0];
    public boolean connected = false;
}
