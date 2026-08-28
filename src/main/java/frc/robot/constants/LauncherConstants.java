package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.launcher.shooter.ShooterTarget;
import frc.robot.subsystems.launcher.turret.TurretAngle;

public final class LauncherConstants {
    public static final Translation2d launcherOffset = new Translation2d(Inches.of(-6), Inches.of(-6));
    public static final Time ballAirTime = Seconds.of(0.8);

    public static final class TurretConstants {
        public static final int canID = 15;
        public static final int channelID = 15;

        public static final SparkMaxConfig motorConfig;
        // public static final Temperature tempWarnThreshold = Celsius.of(75);
        // public static final Temperature thermalShutdownThreshold = Celsius.of(80);

        public static final double mechRotsPerMotorRots = 0.5 / (46.713913 - -19.690401);

        public static final TurretAngle breakAngle = TurretAngle.fromMechanismAngle(Degrees.of(225));
        public static final TurretAngle maxLegal = TurretAngle.fromMechanismAngle(Degrees.of(180));
        public static final TurretAngle minLegal = TurretAngle.fromMechanismAngle(Degrees.of(-90));
        public static final TurretAngle forward = TurretAngle.fromMechanismAngle(Degrees.of(0));
        public static final TurretAngle targetTolerance = TurretAngle.fromMechanismAngle(Degrees.of(2));

        public static final Voltage homingVoltage = Volts.of(-0.2 * 12);
        public static final TurretAngle homingEndPos = TurretAngle.fromMotorRotations(-39.094849);
        public static final Time homingMinRunTime = Seconds.of(0.2);
        public static final Current homingThresholdCurrent = Amps.of(20);
        public static final AngularVelocity homingThresholdVel = RPM.of(1);
        public static final Time homingTimeout = Seconds.of(5);

        static {
            motorConfig = new SparkMaxConfig();
            motorConfig.smartCurrentLimit(4);
            motorConfig.idleMode(IdleMode.kCoast);
            motorConfig.inverted(false);
            motorConfig.voltageCompensation(12);
            motorConfig.closedLoop.outputRange(-1, 1);
            motorConfig.openLoopRampRate(0.1);
            motorConfig.closedLoopRampRate(0.1);

            motorConfig.closedLoop.pid(0.1, 0, 0.1);
        }

        private TurretConstants() {}
    }

    public static final class ShooterConstants {
        public static final int canID = 4;
        public static final int channelID = 4;

        public static final SparkFlexConfig motorConfig;
        public static final Temperature tempWarnThreshold = MotorConstants.neoVortexTempWarnThreshold;
        public static final Temperature thermalShutdownThreshold = MotorConstants.neoVortexThermalShutdownThreshold;

        public static final ShooterTarget targetOffset = ShooterTarget.fromShooterVelocity(RPM.of(100));
        public static final ShooterTarget maxRealTarget = ShooterTarget.fromShooterVelocity(RPM.of(5300));
        public static final ShooterTarget minRealTarget = ShooterTarget.fromDistanceToTarget(Meters.of(1.96));
        public static final ShooterTarget targetTolerance = ShooterTarget.fromShooterVelocity(RPM.of(50));

        static {
            motorConfig = new SparkFlexConfig();
            motorConfig.smartCurrentLimit(40);
            motorConfig.voltageCompensation(12);
            motorConfig.idleMode(IdleMode.kCoast);
            motorConfig.inverted(false);
            motorConfig.closedLoop.outputRange(-1, 1);

            motorConfig.closedLoop.pid(9.3539e-4, 0, 0);
            motorConfig.closedLoop.feedForward.sv(0.11245, 12.071 / 6756d);
        }

        private ShooterConstants() {}
    }

    private LauncherConstants() {}
}
