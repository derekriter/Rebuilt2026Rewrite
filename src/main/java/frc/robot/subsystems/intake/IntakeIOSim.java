package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

public class IntakeIOSim implements IIntakeIO {

    private double rollerVoltage_V = 0;
    private double leftPos = 0;
    private double rightPos = 0;

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerConnected = true;
        inputs.rollerVel_rpm = 6318 * rollerVoltage_V / 11.62;
        inputs.rollerTemp_C = 20;
        inputs.rollerStatorVoltage_V = rollerVoltage_V;
        inputs.rollerStatorCurrent_A = 0;
        inputs.rollerSupplyCurrent_A = 0;
        inputs.rollerThermalShutdown = false;

        inputs.leftDeployerPos = leftPos;

        inputs.rightDeployerPos = rightPos;
    }

    @Override
    public void setRollerVoltage(double voltage_V) {
        rollerVoltage_V = MathUtil.clamp(voltage_V, -12, 12);
    }

    @Override
    public void setLeftDeployerPosition(double pos) {
        leftPos = MathUtil.clamp(pos, 0, 1);
    }

    @Override
    public void setRightDeployerPosition(double pos) {
        rightPos = MathUtil.clamp(pos, 0, 1);
    }
}
