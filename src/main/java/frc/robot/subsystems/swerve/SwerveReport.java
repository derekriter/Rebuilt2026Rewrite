package frc.robot.subsystems.swerve;

public class SwerveReport {
    public boolean isOperational = false;

    public void copyFrom(SwerveReport ref) {
        isOperational = ref.isOperational;
    }
}
