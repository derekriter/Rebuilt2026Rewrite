package frc.robot.brain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import frc.robot.subsystems.launcher.LauncherFlags;

public class RobotState {
    public OpMode opMode = OpMode.DISABLED;
    public boolean isReal = true;

    public boolean isRed = false;
    public boolean autoWinnerIsKnown = false;
    public boolean didWinAuto = false;

    public MutTime modeTime = Seconds.mutable(-1);
    public TeleopPhase phase = TeleopPhase.TRANSITION_SHIFT;
    public MutTime timeLeftInPhase = Seconds.mutable(-1);
    public FieldZone fieldZone = FieldZone.BLUE;

    public LauncherFlags launcherFlags = new LauncherFlags();
    public boolean isTurretHomed = false;

    public TargetingMode targetingMode = TargetingMode.DISABLED;
    public boolean overrideTurret = false;

    public void copyFrom(RobotState ref) {
        opMode = ref.opMode;
        isReal = ref.isReal;

        isRed = ref.isRed;
        autoWinnerIsKnown = ref.autoWinnerIsKnown;
        didWinAuto = ref.didWinAuto;

        modeTime.mut_replace(ref.modeTime);
        phase = ref.phase;
        timeLeftInPhase.mut_replace(ref.timeLeftInPhase);
        fieldZone = ref.fieldZone;

        launcherFlags.copyFrom(ref.launcherFlags);
        isTurretHomed = ref.isTurretHomed;

        targetingMode = ref.targetingMode;
        overrideTurret = ref.overrideTurret;
    }
}
