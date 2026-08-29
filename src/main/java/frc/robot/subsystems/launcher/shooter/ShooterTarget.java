package frc.robot.subsystems.launcher.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.robot.constants.LauncherConstants.ShooterConstants;

public final class ShooterTarget {

    private final MutAngularVelocity vel;

    private ShooterTarget(MutAngularVelocity _vel) {
        vel = _vel;
    }

    // from
    public static ShooterTarget fromShooterVelocity(AngularVelocity vel) {
        return new ShooterTarget(vel.mutableCopy());
    }

    public static ShooterTarget fromShooterRPM(double rpm) {
        return new ShooterTarget(RPM.mutable(rpm));
    }

    public static ShooterTarget fromDistanceToTarget(Distance dist) {
        return new ShooterTarget(RPM.mutable(distanceToVel_rpm(dist.in(Meters))));
    }

    public static ShooterTarget fromMetersToTarget(double meters) {
        return new ShooterTarget(RPM.mutable(distanceToVel_rpm(meters)));
    }

    // as
    public AngularVelocity asShooterVelocity() {
        return vel.copy();
    }

    public double asShooterRPM() {
        return vel.in(RPM);
    }

    public Distance asDistanceToTarget() {
        return velToDistance(vel);
    }

    public double asMetersToTarget() {
        return velToDistance_m(vel.in(RPM));
    }

    // conversions
    public static Distance velToDistance(AngularVelocity vel) {
        return Meters.of(velToDistance_m(vel.in(RPM)));
    }

    public static double velToDistance_m(double vel_rpm) {
        return Math.max((vel_rpm - ShooterConstants.targetOffset.asShooterRPM() - 1614) / 381.0d, 0);
    }

    public static AngularVelocity distanceToVel(Distance dist) {
        return RPM.of(distanceToVel_rpm(dist.in(Meters)));
    }

    public static double distanceToVel_rpm(double dist_m) {
        return 381 * dist_m + 1614 + ShooterConstants.targetOffset.asShooterRPM();
    }

    // tests + operators
    public boolean isLegal() {
        return vel.gte(ShooterConstants.minRealTarget.vel) && vel.lte(ShooterConstants.maxRealTarget.vel);
    }

    public void clampToLegalRange() {
        if (isLegal()) return;

        vel.mut_replace(
                MathUtil.clamp(
                        vel.in(RPM),
                        ShooterConstants.minRealTarget.vel.in(RPM),
                        ShooterConstants.maxRealTarget.vel.in(RPM)),
                RPM);
    }

    public void add(ShooterTarget b) {
        vel.mut_plus(b.vel);
    }
}
