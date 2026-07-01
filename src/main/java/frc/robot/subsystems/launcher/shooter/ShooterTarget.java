package frc.robot.subsystems.launcher.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.config.LauncherConstants.ShooterConstants;

public final class ShooterTarget {

    private final AngularVelocity vel;

    private ShooterTarget(AngularVelocity _vel) {
        vel = _vel;
    }

    public static ShooterTarget fromShooterVelocity(AngularVelocity vel) {
        return new ShooterTarget(vel);
    }

    public static ShooterTarget fromDistanceToTarget(Distance dist) {
        return fromShooterVelocity(
                RPM.of(1614 + 381 * dist.in(Meters)).plus(ShooterConstants.targetOffset.asShooterVelocity()));
    }

    public AngularVelocity asShooterVelocity() {
        return vel;
    }

    public Distance asDistanceToTarget() {
        return Meters.of(
                (vel.minus(ShooterConstants.targetOffset.asShooterVelocity()).in(RPM) - 1614) / 381.0d);
    }

    public ShooterTarget clampToLegalRange() {
        return ShooterTarget.fromShooterVelocity(RPM.of(MathUtil.clamp(
                vel.in(RPM),
                ShooterConstants.minRealTarget.asShooterVelocity().in(RPM),
                ShooterConstants.maxRealTarget.asShooterVelocity().in(RPM))));
    }

    public boolean isLegal() {
        return vel.gte(ShooterConstants.minRealTarget.asShooterVelocity())
                && vel.lte(ShooterConstants.maxRealTarget.asShooterVelocity());
    }

    @Override
    public boolean equals(Object other) {
        if (other.getClass() != ShooterTarget.class) return false;

        return ((ShooterTarget) other).vel.isEquivalent(vel);
    }
}
