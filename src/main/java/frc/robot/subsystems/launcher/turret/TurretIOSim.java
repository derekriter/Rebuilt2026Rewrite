package frc.robot.subsystems.launcher.turret;

public class TurretIOSim implements ITurretIO {

    public TurretIOSim() {}

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = true;
        inputs.pos_rots = 0;
        inputs.vel_RPM = 0;
        inputs.temp_C = 20;
        inputs.appliedOut_perc = 0;
        inputs.statorVoltage_V = 0;
        inputs.statorCurrent_A = 0;
        inputs.thermalShutdown = false;
    }

    @Override
    public void setPositionTarget(double target_rots) {}

    @Override
    public void setVoltage(double voltage_V) {}

    @Override
    public void setEncoderPosition(double position_rots) {}
}
