package frc.robot.subsystems.launcher.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IShooterIO {
    public static final IShooterIO blank = new IShooterIO() {
        @Override
        public void updateInputs(ShooterIOInputs inputs) {}

        @Override
        public void setVelocityTarget(double velocity_rpm) {}

        @Override
        public void setVoltage(double voltage_volts) {}
    };

    public static class ShooterIOInputs implements LoggableInputs {
        public boolean connected = false;
        public double pos_rots = Double.NaN;
        public double vel_RPM = Double.NaN;
        public double temp_C = Double.NaN;
        public double appliedOut_perc = Double.NaN;
        public double statorVoltage_V = Double.NaN;
        public double statorCurrent_A = Double.NaN;

        @Override
        public void toLog(LogTable table) {
            table.put("connected", connected);
            table.put("pos", pos_rots, Rotations.name());
            table.put("vel", vel_RPM, RPM.name());
            table.put("temp", temp_C, Celsius.name());
            table.put("appliedOut", appliedOut_perc);
            table.put("statorVoltage", statorVoltage_V, Volts.name());
            table.put("statorCurrent", statorCurrent_A, Amps.name());
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("connected", connected);
            pos_rots = table.get("pos", pos_rots);
            vel_RPM = table.get("vel", vel_RPM);
            temp_C = table.get("temp", temp_C);
            appliedOut_perc = table.get("appliedOut", appliedOut_perc);
            statorVoltage_V = table.get("statorVoltage", statorVoltage_V);
            statorCurrent_A = table.get("statorCurrent", statorCurrent_A);
        }
    }

    public void updateInputs(ShooterIOInputs inputs);

    public void setVelocityTarget(double velocity_rpm);

    public void setVoltage(double voltage_volts);
}
