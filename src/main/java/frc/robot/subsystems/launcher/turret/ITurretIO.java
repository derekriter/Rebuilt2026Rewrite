package frc.robot.subsystems.launcher.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ITurretIO {

    public static final ITurretIO blank = new ITurretIO() {
        @Override
        public void updateInputs(TurretIOInputs inputs) {}

        @Override
        public void setPositionTarget(double target_rots) {}

        @Override
        public void setVoltage(double voltage_volts) {}

        @Override
        public void stop() {}

        @Override
        public void setEncoderPosition(double position_rots) {}
    };

    public static class TurretIOInputs implements LoggableInputs {
        public boolean connected = false;
        public double pos_rots = Double.NaN;
        public double vel_RPM = Double.NaN;
        public double temp_C = Double.NaN;
        public double appliedOut_perc = Double.NaN;
        public double voltageOut_V = Double.NaN;
        public double currentOut_A = Double.NaN;

        @Override
        public void toLog(LogTable table) {
            table.put("connected", connected);
            table.put("pos", pos_rots, Rotations.name());
            table.put("vel", vel_RPM, RPM.name());
            table.put("temp", temp_C, Celsius.name());
            table.put("appliedOut", appliedOut_perc);
            table.put("voltageOut", voltageOut_V, Volts.name());
            table.put("currentOut", currentOut_A, Amps.name());
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("connected", connected);
            pos_rots = table.get("pos", pos_rots);
            vel_RPM = table.get("vel", vel_RPM);
            temp_C = table.get("temp", temp_C);
            appliedOut_perc = table.get("appliedOut", appliedOut_perc);
            voltageOut_V = table.get("voltageOut", voltageOut_V);
            currentOut_A = table.get("currentOut", currentOut_A);
        }
    }

    public void updateInputs(TurretIOInputs inputs);

    public void setPositionTarget(double target_rots);

    public void setVoltage(double voltage_volts);

    public void stop();

    public void setEncoderPosition(double position_rots);
}
