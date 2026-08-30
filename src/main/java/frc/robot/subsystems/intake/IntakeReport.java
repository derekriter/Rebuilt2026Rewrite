package frc.robot.subsystems.intake;

public final class IntakeReport {

    public boolean rollerOperational = false;
    public boolean rollerStalling = false;

    public boolean deployerOperational = false;
    public boolean hasDeployed = false;

    public void copyFrom(IntakeReport ref) {
        rollerOperational = ref.rollerOperational;
        rollerStalling = ref.rollerStalling;

        deployerOperational = ref.deployerOperational;
        hasDeployed = ref.hasDeployed;
    }
}
