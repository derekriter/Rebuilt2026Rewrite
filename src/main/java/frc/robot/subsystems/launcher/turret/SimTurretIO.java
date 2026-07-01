package frc.robot.subsystems.launcher.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class SimTurretIO implements ITurretIO {

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = true;
        inputs.current = Amps.of(0);
        inputs.motorConfigSuccessfull = true;
        inputs.temp = Celsius.of(20);
        inputs.position = Rotations.of(0);

        inputs.faultEscEEPROM = false;
        inputs.faultFirmware = false;
        inputs.faultGateDriver = false;
        inputs.faultMotorType = false;
        inputs.faultSensor = false;
    }

    @Override
    public void runToPosition(Angle pos) {}

    @Override
    public void dutyCycle(double duty) {}

    @Override
    public void stop() {}

    @Override
    public void setBrake(boolean useBrake) {}
}
