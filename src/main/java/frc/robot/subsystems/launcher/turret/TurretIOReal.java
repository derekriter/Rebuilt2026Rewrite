package frc.robot.subsystems.launcher.turret;

import static frc.robot.constants.LauncherConstants.TurretConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.MotorUtils;

public class TurretIOReal implements ITurretIO {

    private final SparkMax turret;

    public TurretIOReal() {
        turret = new SparkMax(canID, MotorType.kBrushless);
        MotorUtils.safeApplyConfig(
                turret,
                "turret",
                canID,
                channelID,
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // inputs.connected = turret.getLastError() == REVLibError.kOk;
        inputs.connected = MotorUtils.isSparkConnected(turret);

        if (inputs.connected) {
            inputs.pos_rots = turret.getEncoder().getPosition(); // frame 2
            inputs.vel_RPM = turret.getEncoder().getVelocity(); // frame 1
            inputs.temp_C = turret.getMotorTemperature(); // frame 1
            inputs.appliedOut_perc = turret.getAppliedOutput(); // frame 0
            inputs.statorVoltage_V = turret.getBusVoltage() * inputs.appliedOut_perc; // frame 0 & 1
            inputs.statorCurrent_A = turret.getOutputCurrent(); // frame 1
        }
    }

    @Override
    public void setPositionTarget(double target_rots) {
        turret.getClosedLoopController().setSetpoint(target_rots, ControlType.kPosition);
    }

    @Override
    public void setVoltage(double voltage_volts) {
        turret.setVoltage(voltage_volts);
    }

    @Override
    public void setEncoderPosition(double position_rots) {
        turret.getEncoder().setPosition(position_rots);
    }
}
