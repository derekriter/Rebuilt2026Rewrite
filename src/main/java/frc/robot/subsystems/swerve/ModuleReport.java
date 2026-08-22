package frc.robot.subsystems.swerve;

public final class ModuleReport {

    public boolean driveOperational = true;
    public boolean driveThermalShutdown = false;
    public boolean steerOperational = true;
    public boolean steerThermalShutdown = false;
    public boolean encoderOperational = true;

    public void copyFrom(ModuleReport ref) {
        driveOperational = ref.driveOperational;
        driveThermalShutdown = ref.driveThermalShutdown;
        steerOperational = ref.steerOperational;
        steerThermalShutdown = ref.steerThermalShutdown;
        encoderOperational = ref.encoderOperational;
    }
}
