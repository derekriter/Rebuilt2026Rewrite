package frc.robot.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.launcher.ShooterTarget;
import frc.robot.subsystems.launcher.TurretAngle;

public final class LauncherConfig {
    public static final class TurretConfig {

        public static final String systemName = "turret";
        public static final int canID = 15;
        public static final String motorName = "turretMotor";

        public static final SparkMaxConfig motorConfig;

        public static final double motorRotsPerMechRots = (46.713913 - -19.690401) / 0.5;

        public static final TurretAngle forward = TurretAngle.fromMechanismAngle(Degrees.of(0));
        public static final TurretAngle maxLegal = TurretAngle.fromMechanismAngle(Degrees.of(180));
        public static final TurretAngle minLegal = TurretAngle.fromMechanismAngle(Degrees.of(-90));
        public static final TurretAngle breakAngle = TurretAngle.fromMechanismAngle(Degrees.of(225));

        public static final double calibrationSpeed = -0.2;
        public static final TurretAngle calibrationEndPos = TurretAngle.fromMotorRotations(-39.094849);
        public static final Time calibrationEndDelay = Seconds.of(0.2);
        public static final Current calibrationThresholdCurrent = Amps.of(20);
        public static final AngularVelocity calibrationThresholdVel = RPM.of(1);
        public static final Time calibrationTimeout = Seconds.of(5);

        static {
            motorConfig = new SparkMaxConfig();
            motorConfig.smartCurrentLimit(4);
            motorConfig.idleMode(IdleMode.kCoast);
            motorConfig.inverted(false);
            motorConfig.closedLoop.outputRange(-1, 1);
            motorConfig.openLoopRampRate(0.1);
            motorConfig.closedLoopRampRate(0.1);

            motorConfig.closedLoop.pid(0.1, 0, 0.1);
        }

        private TurretConfig() {}
    }

    public static final class ShooterConfig {
        public static final String systemName = "shooter";
        public static final int canID = 4;
        public static final String motorName = "shooterMotor";

        public static final SparkFlexConfig motorConfig;

        public static final ShooterTarget upwardTolerance = ShooterTarget.fromShooterVelocity(RPM.of(100));
        public static final ShooterTarget downwardTolerance = ShooterTarget.fromShooterVelocity(RPM.of(100));
        public static final ShooterTarget maxRealTarget = ShooterTarget.fromShooterVelocity(RPM.of(5300));
        public static final ShooterTarget minRealTarget = ShooterTarget.fromDistanceToTarget(Meters.of(1.96));
        public static final ShooterTarget targetOffset = ShooterTarget.fromShooterVelocity(RPM.of(100));

        static {
            motorConfig = new SparkFlexConfig();
            motorConfig.smartCurrentLimit(40);
            motorConfig.idleMode(IdleMode.kCoast);
            motorConfig.inverted(false);
            motorConfig.closedLoop.outputRange(-1, 1);

            motorConfig.closedLoop.pid(9.3539e-4, 0, 0);
            motorConfig.closedLoop.feedForward.sv(0.11245, 12.071 / 6756d);
        }

        private ShooterConfig() {}
    }

    public static final Translation2d launcherOffset = new Translation2d(Inches.of(-6), Inches.of(-6));
    public static final Time ballAirTime = Seconds.of(0.8);

    private LauncherConfig() {}
}
