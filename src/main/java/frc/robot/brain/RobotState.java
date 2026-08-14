package frc.robot.brain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import frc.robot.subsystems.launcher.LauncherReport;

public final class RobotState {
    public OpMode opMode = OpMode.DISABLED;
    public boolean isReal = true;
    public boolean isDSAttached = false;

    public boolean isRed = false;
    public boolean autoWinnerIsKnown = false;
    public boolean didWinAuto = false;

    public MutTime modeTime = Seconds.mutable(Double.NaN);
    public TeleopPhase phase = TeleopPhase.TRANSITION_SHIFT;
    public MutTime timeLeftInPhase = Seconds.mutable(Double.NaN);
    public FieldZone fieldZone = FieldZone.BLUE;
    public boolean isHubActive = false;

    public LauncherReport launcherReport = new LauncherReport();
    public boolean isTurretHomed = false;

    public TargetingMode targetingMode = TargetingMode.DISABLED;
    public boolean overrideTurret = false;
    public LEDsMode ledsMode = LEDsMode.DISCONNECTED;

    public void copyFrom(RobotState ref) {
        opMode = ref.opMode;
        isReal = ref.isReal;
        isDSAttached = ref.isDSAttached;

        isRed = ref.isRed;
        autoWinnerIsKnown = ref.autoWinnerIsKnown;
        didWinAuto = ref.didWinAuto;

        modeTime.mut_replace(ref.modeTime);
        phase = ref.phase;
        timeLeftInPhase.mut_replace(ref.timeLeftInPhase);
        fieldZone = ref.fieldZone;
        isHubActive = ref.isHubActive;

        launcherReport.copyFrom(ref.launcherReport);
        isTurretHomed = ref.isTurretHomed;

        targetingMode = ref.targetingMode;
        overrideTurret = ref.overrideTurret;
        ledsMode = ref.ledsMode;
    }
}
