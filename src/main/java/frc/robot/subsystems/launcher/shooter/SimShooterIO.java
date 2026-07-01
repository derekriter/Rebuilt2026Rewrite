package frc.robot.subsystems.launcher.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SimShooterIO implements IShooterIO {

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.connected = true;
        inputs.current = Amps.of(0);
        inputs.motorConfigSuccessfull = true;
        inputs.temp = Celsius.of(20);
        inputs.velocity = RPM.of(0);

        inputs.faultEscEEPROM = false;
        inputs.faultFirmware = false;
        inputs.faultGateDriver = false;
        inputs.faultMotorType = false;
        inputs.faultSensor = false;
    }

    @Override
    public void voltageOut(Voltage volts) {}

    @Override
    public void runToVelocity(AngularVelocity vel) {}

    @Override
    public void stop() {}
}
