package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.IntakeConstants.LeftDeployerConstants;
import frc.robot.constants.IntakeConstants.RightDeployerConstants;
import frc.robot.constants.IntakeConstants.RollerConstants;
import frc.robot.util.MotorUtils;

public class IntakeIOReal implements IIntakeIO {

    private final TalonFX roller;
    private final StatusSignal<Angle> rollerPos_rots;
    private final StatusSignal<AngularVelocity> rollerVel_rps;
    private final StatusSignal<Temperature> rollerTemp_C;
    private final StatusSignal<Voltage> rollerStatorVoltage_V;
    private final StatusSignal<Current> rollerStatorCurrent_A;
    private final StatusSignal<Current> rollerSupplyCurrent_A;
    private final Debouncer rollerConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    private final Servo leftDeployer;
    private final Servo rightDeployer;

    public IntakeIOReal() {
        roller = new TalonFX(RollerConstants.canID);
        MotorUtils.safeApplyConfig(
                roller, "roller", RollerConstants.canID, RollerConstants.channelID, RollerConstants.motorConfig);

        rollerPos_rots = roller.getPosition();
        rollerVel_rps = roller.getVelocity();
        rollerTemp_C = roller.getDeviceTemp();
        rollerStatorVoltage_V = roller.getMotorVoltage();
        rollerStatorCurrent_A = roller.getStatorCurrent();
        rollerSupplyCurrent_A = roller.getSupplyCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                rollerPos_rots,
                rollerVel_rps,
                rollerTemp_C,
                rollerStatorVoltage_V,
                rollerStatorCurrent_A,
                rollerSupplyCurrent_A);
        roller.optimizeBusUtilization();

        leftDeployer = new Servo(LeftDeployerConstants.port);
        rightDeployer = new Servo(RightDeployerConstants.port);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        StatusCode rollerStatus = BaseStatusSignal.refreshAll(
                rollerPos_rots,
                rollerVel_rps,
                rollerTemp_C,
                rollerStatorVoltage_V,
                rollerStatorCurrent_A,
                rollerSupplyCurrent_A);

        inputs.rollerConnected = rollerConnectedDebouncer.calculate(rollerStatus.isOK());
        inputs.rollerPos_rots = rollerPos_rots.getValueAsDouble();
        inputs.rollerVel_rpm = rollerVel_rps.getValueAsDouble() * 60;
        inputs.rollerTemp_C = rollerTemp_C.getValueAsDouble();
        inputs.rollerStatorVoltage_V = rollerStatorVoltage_V.getValueAsDouble();
        inputs.rollerStatorCurrent_A = rollerStatorCurrent_A.getValueAsDouble();
        inputs.rollerSupplyCurrent_A = rollerSupplyCurrent_A.getValueAsDouble();

        inputs.leftDeployerPos = leftDeployer.get();

        inputs.rightDeployerPos = rightDeployer.get();
    }

    @Override
    public void setRollerVoltage(double voltage_V) {
        roller.setVoltage(voltage_V);
    }

    @Override
    public void setLeftDeployerPosition(double pos) {
        leftDeployer.set(pos);
    }

    @Override
    public void setRightDeployerPosition(double pos) {
        rightDeployer.set(pos);
    }
}
