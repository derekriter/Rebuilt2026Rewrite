package frc.robot.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.launcher.shooter.ShooterTarget;
import frc.robot.subsystems.launcher.turret.TurretAngle;

public final class LauncherConstants {
    public static final class TurretConstants {
        public static final String systemName = "turret";
        public static final int canID = 15;
        public static final String motorName = "turretMotor";
        public static final double motorRotsPerMechRots = (46.713913 - -19.690401) / 0.5;

        public static final Current maxCurrent = Amps.of(4);
        public static final boolean coast = true;
        public static final boolean inverted = false;
        public static final double maxDuty = 1;
        public static final Time rampTime = Seconds.of(0.1);

        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0.1;

        public static final TurretAngle forward = TurretAngle.fromMechanismAngle(Degrees.of(0));
        public static final TurretAngle maxLegal = TurretAngle.fromMechanismAngle(Degrees.of(180));
        public static final TurretAngle minLegal = TurretAngle.fromMechanismAngle(Degrees.of(-90));
        public static final TurretAngle breakAngle = TurretAngle.fromMechanismAngle(Degrees.of(225));
    }

    public static final class ShooterConstants {
        public static final String systemName = "shooter";
        public static final int canID = 4;
        public static final String motorName = "shooterMotor";

        public static final Current maxCurrent = Amps.of(40);
        public static final boolean coast = true;
        public static final boolean inverted = false;
        public static final double maxDuty = 1;

        public static final double kP = 9.3539e-4;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.11245;
        public static final double kV = 12.071 / 6756d;

        public static final ShooterTarget upwardTolerance = ShooterTarget.fromShooterVelocity(RPM.of(100));
        public static final ShooterTarget downwardTolerance = ShooterTarget.fromShooterVelocity(RPM.of(100));
        public static final ShooterTarget maxRealTarget = ShooterTarget.fromShooterVelocity(RPM.of(5300));
        public static final ShooterTarget minRealTarget = ShooterTarget.fromDistanceToTarget(Meters.of(1.96));
        public static final ShooterTarget targetOffset = ShooterTarget.fromShooterVelocity(RPM.of(100));
    }

    public static final Translation2d launcherOffset = new Translation2d(Inches.of(-6), Inches.of(-6));
    public static final Time ballAirTime = Seconds.of(0.8);
}
