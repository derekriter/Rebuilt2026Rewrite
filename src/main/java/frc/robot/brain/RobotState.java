package frc.robot.brain;

import frc.robot.subsystems.intake.IntakeReport;
import frc.robot.subsystems.launcher.LauncherReport;
import frc.robot.subsystems.swerve.SwerveReport;

public final class RobotState {
    public OpMode opMode = OpMode.DISABLED;
    public boolean isReal = true;
    public boolean isDSAttached = false;
    public boolean isFMSAttached = false;
    public boolean isBrownedOut = false;

    public boolean isRed = false;
    public boolean autoWinnerIsKnown = false;
    public boolean didWinAuto = false;

    public double modeTime_s = Double.NaN;
    public TeleopPhase phase = TeleopPhase.TRANSITION_SHIFT;
    public double timeLeftInPhase_s = Double.NaN;
    public FieldZone fieldZone = FieldZone.BLUE;
    public boolean isHubActive = false;

    public LauncherReport launcherReport = new LauncherReport();
    public boolean isTurretHomed = false;
    public SwerveReport swerveReport = new SwerveReport();
    public IntakeReport intakeReport = new IntakeReport();

    public TargetingMode targetingMode = TargetingMode.DISABLED;
    public boolean overrideTurret = false;
    public LEDsMode ledsMode = LEDsMode.DISCONNECTED;
    public DriveMode driveMode = DriveMode.DISABLED;
    public boolean shouldDeployIntake = false;
    public RumbleMode driver1RumbleMode = RumbleMode.IDLE;
    public RumbleMode driver2RumbleMode = RumbleMode.IDLE;

    public void copyFrom(RobotState ref) {
        opMode = ref.opMode;
        isReal = ref.isReal;
        isDSAttached = ref.isDSAttached;
        isFMSAttached = ref.isFMSAttached;
        isBrownedOut = ref.isBrownedOut;

        isRed = ref.isRed;
        autoWinnerIsKnown = ref.autoWinnerIsKnown;
        didWinAuto = ref.didWinAuto;

        modeTime_s = ref.modeTime_s;
        phase = ref.phase;
        timeLeftInPhase_s = ref.timeLeftInPhase_s;
        fieldZone = ref.fieldZone;
        isHubActive = ref.isHubActive;

        launcherReport.copyFrom(ref.launcherReport);
        isTurretHomed = ref.isTurretHomed;
        swerveReport.copyFrom(ref.swerveReport);
        intakeReport.copyFrom(ref.intakeReport);

        targetingMode = ref.targetingMode;
        overrideTurret = ref.overrideTurret;
        ledsMode = ref.ledsMode;
        driveMode = ref.driveMode;
        shouldDeployIntake = ref.shouldDeployIntake;
        driver1RumbleMode = ref.driver1RumbleMode;
        driver2RumbleMode = ref.driver2RumbleMode;
    }
}
