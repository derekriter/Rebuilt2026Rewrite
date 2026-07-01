package frc.robot.subsystems.launcher.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.config.LauncherConstants.ShooterConstants;
import frc.robot.util.MotorUtils;

public class SparkFlexShooterIO implements IShooterIO {

    private final SparkFlex motor;
    private final boolean configSuccess;

    public SparkFlexShooterIO() {
        motor = new SparkFlex(ShooterConstants.canID, MotorType.kBrushless);

        var config = new SparkFlexConfig();
        config.smartCurrentLimit((int) ShooterConstants.maxCurrent.in(Amps));
        config.idleMode(ShooterConstants.coast ? IdleMode.kCoast : IdleMode.kBrake);
        config.inverted(ShooterConstants.inverted);
        config.closedLoop.outputRange(-ShooterConstants.maxDuty, ShooterConstants.maxDuty);

        config.closedLoop.pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        config.closedLoop.feedForward.sv(ShooterConstants.kS, ShooterConstants.kV);

        configSuccess = MotorUtils.safeApplyConfig(
                motor,
                ShooterConstants.motorName,
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.connected = !motor.getFaults().can;
        inputs.current = Amps.of(motor.getOutputCurrent());
        inputs.motorConfigSuccessfull = configSuccess;
        inputs.temp = Celsius.of(motor.getMotorTemperature());
        inputs.velocity = RPM.of(motor.getEncoder().getVelocity());

        Faults faults = motor.getFaults();
        inputs.faultEscEEPROM = faults.escEeprom;
        inputs.faultFirmware = faults.firmware;
        inputs.faultGateDriver = faults.gateDriver;
        inputs.faultMotorType = faults.motorType;
        inputs.faultSensor = faults.sensor;
    }

    @Override
    public void voltageOut(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void runToVelocity(AngularVelocity vel) {
        motor.getClosedLoopController().setSetpoint(vel.in(RPM), ControlType.kVelocity);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
