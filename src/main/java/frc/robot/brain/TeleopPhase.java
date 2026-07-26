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

    public static TeleopPhase fromTeleopTimer(Time teleopTime) {
        if (teleopTime.lt(TRANSITION_SHIFT.endTime)) return TRANSITION_SHIFT;
        if (teleopTime.lt(SHIFT1.endTime)) return SHIFT1;
        if (teleopTime.lt(SHIFT2.endTime)) return SHIFT2;
        if (teleopTime.lt(SHIFT3.endTime)) return SHIFT3;
        if (teleopTime.lt(SHIFT4.endTime)) return SHIFT4;
        return ENDGAME;
    }

    public Time getTimeRemaining(Time teleopTime) {
        return this.endTime.minus(teleopTime);
    }

    public boolean isHubEnabled(boolean didWinAuto) {
        return switch (this) {
            case TRANSITION_SHIFT, ENDGAME -> true;
            case SHIFT1, SHIFT3 -> !didWinAuto;
            case SHIFT2, SHIFT4 -> didWinAuto;
        };
    }
}
