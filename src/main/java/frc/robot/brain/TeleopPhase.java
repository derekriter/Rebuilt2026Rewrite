package frc.robot.brain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public enum TeleopPhase {
    TRANSITION_SHIFT(Seconds.of(10)),
    SHIFT1(Seconds.of(10 + 1 * 25)),
    SHIFT2(Seconds.of(10 + 2 * 25)),
    SHIFT3(Seconds.of(10 + 3 * 25)),
    SHIFT4(Seconds.of(10 + 4 * 25)),
    ENDGAME(Seconds.of(10 + 4 * 25 + 30));

    public final Time endTime;

    private TeleopPhase(Time _endTime) {
        endTime = _endTime;
    }

    public static TeleopPhase fromTeleopTime(double teleopTime_s) {
        if (teleopTime_s < TRANSITION_SHIFT.endTime.in(Seconds)) return TRANSITION_SHIFT;
        if (teleopTime_s < SHIFT1.endTime.in(Seconds)) return SHIFT1;
        if (teleopTime_s < SHIFT2.endTime.in(Seconds)) return SHIFT2;
        if (teleopTime_s < SHIFT3.endTime.in(Seconds)) return SHIFT3;
        if (teleopTime_s < SHIFT4.endTime.in(Seconds)) return SHIFT4;
        return ENDGAME;
    }

    public double getTimeRemaining_s(double teleopTime_s) {
        return this.endTime.in(Seconds) - teleopTime_s;
    }

    public boolean isHubEnabled(boolean didWinAuto) {
        return switch (this) {
            case TRANSITION_SHIFT, ENDGAME -> true;
            case SHIFT1, SHIFT3 -> !didWinAuto;
            case SHIFT2, SHIFT4 -> didWinAuto;
        };
    }
}
