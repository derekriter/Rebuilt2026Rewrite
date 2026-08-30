package frc.robot.subsystems.intake;

public class IntakeIOSim implements IIntakeIO {

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerConnected = true;
        inputs.rollerPos_rots = 0;
        inputs.rollerVel_rpm = 0;
        inputs.rollerTemp_C = 20;
        inputs.rollerStatorVoltage_V = 0;
        inputs.rollerStatorCurrent_A = 0;
        inputs.rollerSupplyCurrent_A = 0;
    }

    @Override
    public void setRollerVoltage(double voltage_V) {}

    @Override
    public void setLeftDeployerPosition(double pos) {}

    @Override
    public void setRightDeployerPosition(double pos) {}
}
