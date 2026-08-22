package frc.robot.telemetry.writer.compound;

import frc.robot.brain.RobotState;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.telemetry.writer.StringWriter;

public class RobotStateWriter implements AutoCloseable {

    private final StringWriter opModeWriter;
    private final BoolWriter isRealWriter;
    private final BoolWriter isDSAttachedWriter;
    private final BoolWriter isBrownedOutWriter;

    private final BoolWriter isRedWriter;
    private final BoolWriter autoWinnerIsKnownWriter;
    private final BoolWriter didWinAutoWriter;

    private final DoubleWriter modeTimeWriter;
    private final StringWriter phaseWriter;
    private final DoubleWriter timeLeftInPhaseWriter;
    private final StringWriter fieldZoneWriter;
    private final BoolWriter isHubActiveWriter;

    private final LauncherReportWriter launcherReportWriter;
    private final BoolWriter isTurretHomedWriter;
    private final SwerveReportWriter swerveReportWriter;

    private final StringWriter targetingModeWriter;
    private final BoolWriter overrideTurretWriter;
    private final StringWriter ledsModeWriter;
    private final StringWriter driveModeWriter;

    private final RobotState nullFallback;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public RobotStateWriter(String table, RobotState _nullFallback) {
        opModeWriter = Telemetry.makeStringWriterEx(table, "opMode", null, true, false);
        isRealWriter = Telemetry.makeBoolWriterEx(table, "isReal", null, true, false);
        isDSAttachedWriter = Telemetry.makeBoolWriterEx(table, "isDSAttached", null, true, false);
        isBrownedOutWriter = Telemetry.makeBoolWriterEx(table, "isBrowedOut", null, true, false);

        isRedWriter = Telemetry.makeBoolWriterEx(table, "isRed", null, true, false);
        autoWinnerIsKnownWriter = Telemetry.makeBoolWriterEx(table, "autoWinnerIsKnown", null, true, false);
        didWinAutoWriter = Telemetry.makeBoolWriterEx(table, "didWinAuto", null, true, false);

        modeTimeWriter = Telemetry.makeDoubleWriterEx(table, "modeTime", TelemetryUnits.seconds, false, true);
        phaseWriter = Telemetry.makeStringWriterEx(table, "phase", null, true, false);
        timeLeftInPhaseWriter =
                Telemetry.makeDoubleWriterEx(table, "timeLeftInPhase", TelemetryUnits.seconds, true, true);
        fieldZoneWriter = Telemetry.makeStringWriterEx(table, "fieldZone", null, true, false);
        isHubActiveWriter = Telemetry.makeBoolWriterEx(table, "isHubActive", null, true, false);

        launcherReportWriter = Telemetry.makeLauncherReportWriter(table, "launcherReport");
        isTurretHomedWriter = Telemetry.makeBoolWriterEx(table, "isTurretHomed", null, true, false);
        swerveReportWriter = Telemetry.makeSwerveReportWriter(table, "swerveReport");

        targetingModeWriter = Telemetry.makeStringWriterEx(table, "targetingMode", null, true, false);
        overrideTurretWriter = Telemetry.makeBoolWriterEx(table, "overrideTurret", null, true, false);
        ledsModeWriter = Telemetry.makeStringWriterEx(table, "ledsMode", null, true, false);
        driveModeWriter = Telemetry.makeStringWriterEx(table, "driveMode", null, true, false);

        nullFallback = _nullFallback;
    }

    public void set(RobotState state_nl) {
        if (state_nl == null) state_nl = nullFallback;

        opModeWriter.set(state_nl.opMode.name());
        isRealWriter.set(state_nl.isReal);
        isDSAttachedWriter.set(state_nl.isDSAttached);
        isBrownedOutWriter.set(state_nl.isBrownedOut);

        isRedWriter.set(state_nl.isRed);
        autoWinnerIsKnownWriter.set(state_nl.autoWinnerIsKnown);
        didWinAutoWriter.set(state_nl.didWinAuto);

        modeTimeWriter.set(state_nl.modeTime_s);
        phaseWriter.set(state_nl.phase.name());
        timeLeftInPhaseWriter.set(state_nl.timeLeftInPhase_s);
        fieldZoneWriter.set(state_nl.fieldZone.name());
        isHubActiveWriter.set(state_nl.isHubActive);

        launcherReportWriter.set(state_nl.launcherReport);
        isTurretHomedWriter.set(state_nl.isTurretHomed);
        swerveReportWriter.set(state_nl.swerveReport);

        targetingModeWriter.set(state_nl.targetingMode.name());
        overrideTurretWriter.set(state_nl.overrideTurret);
        ledsModeWriter.set(state_nl.ledsMode.name());
        driveModeWriter.set(state_nl.driveMode.name());
    }

    @Override
    public void close() {
        opModeWriter.close();
        isRealWriter.close();
        isDSAttachedWriter.close();
        isBrownedOutWriter.close();
        isRedWriter.close();
        autoWinnerIsKnownWriter.close();
        didWinAutoWriter.close();
        modeTimeWriter.close();
        phaseWriter.close();
        timeLeftInPhaseWriter.close();
        fieldZoneWriter.close();
        isHubActiveWriter.close();
        launcherReportWriter.close();
        isTurretHomedWriter.close();
        swerveReportWriter.close();
        targetingModeWriter.close();
        overrideTurretWriter.close();
        ledsModeWriter.close();
        driveModeWriter.close();
    }
}
