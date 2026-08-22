package frc.robot.subsystems.swerve;

public class SwerveReport {
    public boolean isOperational = true;

    public void copyFrom(SwerveReport ref) {
        isOperational = ref.isOperational;
    }
}
