package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IIntakeIO {
    public static final IIntakeIO blank = new IIntakeIO() {
        @Override
        public void updateInputs(IntakeIOInputs inputs) {}

        @Override
        public void setRollerVoltage(double voltage_V) {}

        @Override
        public void setLeftDeployerPosition(double pos) {}

        @Override
        public void setRightDeployerPosition(double pos) {}
    };

    public static class IntakeIOInputs implements LoggableInputs {
        public boolean rollerConnected = false;
        public double rollerPos_rots = Double.NaN;
        public double rollerVel_rpm = Double.NaN;
        public double rollerTemp_C = Double.NaN;
        public double rollerStatorVoltage_V = Double.NaN;
        public double rollerStatorCurrent_A = Double.NaN;
        public double rollerSupplyCurrent_A = Double.NaN;

        public double leftDeployerPos = Double.NaN;

        public double rightDeployerPos = Double.NaN;

        @Override
        public void toLog(LogTable table) {
            table.put("rollerConnected", rollerConnected);
            table.put("rollerPos", rollerPos_rots, Rotations.name());
            table.put("rollerVel", rollerVel_rpm, RPM.name());
            table.put("rollerTemp", rollerTemp_C, Celsius.name());
            table.put("rollerStatorVoltage", rollerStatorVoltage_V, Volts.name());
            table.put("rollerStatorCurrent", rollerStatorCurrent_A, Amps.name());
            table.put("rollerSupplyCurrent", rollerSupplyCurrent_A, Amps.name());

            table.put("leftDeployerPos", leftDeployerPos);

            table.put("rightDeployerPos", rightDeployerPos);
        }

        @Override
        public void fromLog(LogTable table) {
            rollerConnected = table.get("rollerConnected", rollerConnected);
            rollerPos_rots = table.get("rollerPos", rollerPos_rots);
            rollerVel_rpm = table.get("rollerVel", rollerVel_rpm);
            rollerTemp_C = table.get("rollerTemp", rollerTemp_C);
            rollerStatorVoltage_V = table.get("rollerStatorVoltage", rollerStatorVoltage_V);
            rollerStatorCurrent_A = table.get("rollerStatorCurrent", rollerStatorCurrent_A);
            rollerSupplyCurrent_A = table.get("rollerSupplyCurrent", rollerSupplyCurrent_A);

            leftDeployerPos = table.get("leftDeployerPos", leftDeployerPos);

            rightDeployerPos = table.get("rightDeployerPos", rightDeployerPos);
        }
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setRollerVoltage(double voltage_V);

    public void setLeftDeployerPosition(double pos);

    public void setRightDeployerPosition(double pos);
}
