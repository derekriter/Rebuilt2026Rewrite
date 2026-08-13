package frc.robot.config;

import edu.wpi.first.units.measure.Time;
import frc.robot.telemetry.TelemetryLevel;
import java.util.Optional;

public final class TelemetryConfig {
    public static final String PREFIX = "[telem] ";
    public static final TelemetryLevel telemetryLevel = TelemetryLevel.ENABLED;
    public static final boolean defaultIncludeNTInChecks = true;
    public static final boolean defaultDisableChecks = false;

    // public static final Optional<Time> loopOverrunPeriod = Optional.of(Seconds.of(0.2));
    public static final Optional<Time> loopOverrunPeriod = Optional.empty();
    public static final boolean showJoystickDisconnectWarnings = false;
    public static final boolean ctreLoggingEnabled = false;
    public static final boolean revLoggingEnabled = false;

    private TelemetryConfig() {}
}
