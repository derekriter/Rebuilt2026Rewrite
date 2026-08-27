package frc.robot.subsystems.launcher.shooter;

public class ShooterIOSim implements IShooterIO {

    public ShooterIOSim() {}

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.connected = true;
        inputs.pos_rots = 0;
        inputs.vel_RPM = 0;
        inputs.temp_C = 20;
        inputs.appliedOut_perc = 0;
        inputs.voltageOut_V = 0;
        inputs.currentOut_A = 0;
    }

    @Override
    public void setVelocityTarget(double velocity_rpm) {}

    @Override
    public void setVoltage(double voltage_volts) {}

    @Override
    public void stop() {}
}
