package frc.robot.telemetry.writer.compound;

import static edu.wpi.first.units.Units.Seconds;

import frc.robot.brain.RobotState;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.telemetry.writer.StringWriter;

public class RobotStateWriter implements AutoCloseable {

    private final StringWriter opModeWriter;
    private final BoolWriter isRealWriter;

    private final BoolWriter isRedWriter;
    private final BoolWriter autoWinnerIsKnownWriter;
    private final BoolWriter didWinAutoWriter;

    private final DoubleWriter modeTimeWriter;
    private final StringWriter phaseWriter;
    private final DoubleWriter timeLeftInPhaseWriter;
    private final StringWriter fieldZoneWriter;

    private final LauncherFlagsWriter launcherFlagsWriter;
    private final BoolWriter isTurretHomedWriter;

    private final StringWriter targetingModeWriter;
    private final BoolWriter overrideTurretWriter;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public RobotStateWriter(String table) {
        opModeWriter = Telemetry.makeStringWriterEx(table, "opMode", null, true, false);
        isRealWriter = Telemetry.makeBoolWriterEx(table, "isReal", null, true, false);

        isRedWriter = Telemetry.makeBoolWriterEx(table, "isRed", null, true, false);
        autoWinnerIsKnownWriter = Telemetry.makeBoolWriterEx(table, "autoWinnerIsKnown", null, true, false);
        didWinAutoWriter = Telemetry.makeBoolWriterEx(table, "didWinAuto", null, true, false);

        modeTimeWriter = Telemetry.makeDoubleWriterEx(table, "modeTime", TelemetryUnits.seconds, false, true);
        phaseWriter = Telemetry.makeStringWriterEx(table, "phase", null, true, false);
        timeLeftInPhaseWriter =
                Telemetry.makeDoubleWriterEx(table, "timeLeftInPhase", TelemetryUnits.seconds, true, true);
        fieldZoneWriter = Telemetry.makeStringWriterEx(table, "fieldZone", null, true, false);

        launcherFlagsWriter = Telemetry.makeLauncherFlagsWriter(table, "launcherFlags");
        isTurretHomedWriter = Telemetry.makeBoolWriterEx(table, "isTurretHomed", null, true, false);

        targetingModeWriter = Telemetry.makeStringWriterEx(table, "targetingMode", null, true, false);
        overrideTurretWriter = Telemetry.makeBoolWriterEx(table, "overrideTurret", null, true, false);
    }

    public void update(RobotState state) {
        opModeWriter.set(state.opMode.name());
        isRealWriter.set(state.isReal);

        isRedWriter.set(state.isRed);
        autoWinnerIsKnownWriter.set(state.autoWinnerIsKnown);
        didWinAutoWriter.set(state.didWinAuto);

        modeTimeWriter.set(state.modeTime.in(Seconds));
        phaseWriter.set(state.phase.name());
        timeLeftInPhaseWriter.set(state.timeLeftInPhase.in(Seconds));
        fieldZoneWriter.set(state.fieldZone.name());

        launcherFlagsWriter.update(state.launcherFlags);
        isTurretHomedWriter.set(state.isTurretHomed);

        targetingModeWriter.set(state.targetingMode.name());
        overrideTurretWriter.set(state.overrideTurret);
    }

    @Override
    public void close() {
        opModeWriter.close();
        isRealWriter.close();
        isRedWriter.close();
        autoWinnerIsKnownWriter.close();
        didWinAutoWriter.close();
        modeTimeWriter.close();
        phaseWriter.close();
        timeLeftInPhaseWriter.close();
        fieldZoneWriter.close();
        launcherFlagsWriter.close();
        isTurretHomedWriter.close();
        targetingModeWriter.close();
        overrideTurretWriter.close();
    }
}
