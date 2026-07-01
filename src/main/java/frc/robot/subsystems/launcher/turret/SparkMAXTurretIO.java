package frc.robot.subsystems.launcher.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import frc.robot.config.LauncherConstants.TurretConstants;
import frc.robot.util.MotorUtils;

public class SparkMAXTurretIO implements ITurretIO {

    private final SparkMax motor;
    private final boolean configSuccess;
    private boolean brakeActive = !TurretConstants.coast;

    public SparkMAXTurretIO() {
        motor = new SparkMax(TurretConstants.canID, MotorType.kBrushless);

        var config = new SparkMaxConfig();
        config.smartCurrentLimit((int) (TurretConstants.maxCurrent.in(Amps)));
        config.idleMode(TurretConstants.coast ? IdleMode.kCoast : IdleMode.kBrake);
        config.inverted(TurretConstants.inverted);
        config.closedLoop.outputRange(-TurretConstants.maxDuty, TurretConstants.maxDuty);
        config.openLoopRampRate(TurretConstants.rampTime.in(Seconds));
        config.closedLoopRampRate(TurretConstants.rampTime.in(Seconds));

        config.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);

        configSuccess = MotorUtils.safeApplyConfig(
                motor,
                TurretConstants.motorName,
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = !motor.getFaults().can;
        inputs.current = Amps.of(motor.getOutputCurrent());
        inputs.motorConfigSuccessfull = configSuccess;
        inputs.temp = Celsius.of(motor.getMotorTemperature());
        inputs.position = Rotations.of(motor.getEncoder().getPosition());

        Faults faults = motor.getFaults();
        inputs.faultEscEEPROM = faults.escEeprom;
        inputs.faultFirmware = faults.firmware;
        inputs.faultGateDriver = faults.gateDriver;
        inputs.faultMotorType = faults.motorType;
        inputs.faultSensor = faults.sensor;
    }

    @Override
    public void runToPosition(Angle pos) {
        motor.getClosedLoopController().setSetpoint(pos.in(Rotations), ControlType.kPosition);
    }

    @Override
    public void dutyCycle(double duty) {
        motor.set(duty);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setBrake(boolean useBrake) {
        if (brakeActive == useBrake) return; // avoid unnecessary configurations

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(useBrake ? IdleMode.kBrake : IdleMode.kCoast);

        MotorUtils.safeApplyConfig(
                motor,
                TurretConstants.motorName,
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters,
                1);
        brakeActive = useBrake;
    }
}
