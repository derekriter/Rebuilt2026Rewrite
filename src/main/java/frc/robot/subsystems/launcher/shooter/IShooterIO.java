package frc.robot.subsystems.launcher.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IShooterIO {

    public static final IShooterIO blank = new IShooterIO() {
        @Override
        public void updateInputs(ShooterIOInputs inputs) {}

        @Override
        public void voltageOut(Voltage volts) {}

        @Override
        public void runToVelocity(AngularVelocity vel) {}

        @Override
        public void stop() {}
    };

    @AutoLog
    public static class ShooterIOInputs {
        public boolean connected;
        public Temperature temp;
        public Current current;
        public boolean motorConfigSuccessfull;
        public AngularVelocity velocity;
        public boolean faultEscEEPROM;
        public boolean faultFirmware;
        public boolean faultGateDriver;
        public boolean faultSensor;
        public boolean faultMotorType;
    }

    public void updateInputs(ShooterIOInputs inputs);

    public void voltageOut(Voltage volts);

    public void runToVelocity(AngularVelocity vel);

    public void stop();
}
