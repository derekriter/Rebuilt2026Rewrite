package frc.robot.subsystems.launcher.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface ITurretIO {

    public static final ITurretIO blank = new ITurretIO() {
        @Override
        public void updateInputs(TurretIOInputs inputs) {}

        @Override
        public void runToPosition(Angle pos) {}

        @Override
        public void dutyCycle(double duty) {}

        @Override
        public void stop() {}

        @Override
        public void setBrake(boolean useBrake) {}
    };

    @AutoLog
    public static class TurretIOInputs {
        public boolean connected;
        public Temperature temp;
        public Current current;
        public boolean motorConfigSuccessfull;
        public Angle position;
        public boolean faultEscEEPROM;
        public boolean faultFirmware;
        public boolean faultGateDriver;
        public boolean faultSensor;
        public boolean faultMotorType;
    }

    public void updateInputs(TurretIOInputs inputs);

    public void runToPosition(Angle pos);

    public void dutyCycle(double duty);

    public void stop();

    public void setBrake(boolean useBrake);
}
