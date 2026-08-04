package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.robot.config.LauncherConfig.ShooterConfig;

public final class ShooterTarget {

    private final MutAngularVelocity vel;

    private ShooterTarget(MutAngularVelocity _vel) {
        vel = _vel;
    }

    public static ShooterTarget fromShooterVelocity(AngularVelocity vel) {
        return new ShooterTarget(vel.mutableCopy());
    }

    public static ShooterTarget fromShooterRPM(double rpm) {
        return new ShooterTarget(RPM.mutable(rpm));
    }

    public static ShooterTarget fromDistanceToTarget(Distance dist) {
        return new ShooterTarget(RPM.mutable(381 * dist.in(Meters) + 1614 + ShooterConfig.targetOffset.asShooterRPM()));
    }

    public static ShooterTarget fromMetersToTarget(double meters) {
        return new ShooterTarget(RPM.mutable(381 * meters + 1614 + ShooterConfig.targetOffset.asShooterRPM()));
    }

    public AngularVelocity asShooterVelocity() {
        return vel.copy();
    }

    public double asShooterRPM() {
        return vel.in(RPM);
    }

    public Distance asDistanceToTarget() {
        return Meters.of((vel.in(RPM) - ShooterConfig.targetOffset.asShooterRPM() - 1614) / 381.0d);
    }

    public double asMetersToTarget() {
        return (vel.in(RPM) - ShooterConfig.targetOffset.asShooterRPM() - 1614) / 381.0d;
    }

    public boolean isLegal() {
        return vel.gte(ShooterConfig.minRealTarget.vel) && vel.lte(ShooterConfig.maxRealTarget.vel);
    }

    public void clampToLegalRange() {
        if (isLegal()) return;

        vel.mut_replace(
                MathUtil.clamp(
                        vel.in(RPM), ShooterConfig.minRealTarget.vel.in(RPM), ShooterConfig.maxRealTarget.vel.in(RPM)),
                RPM);
    }

    public void add(ShooterTarget b) {
        vel.mut_plus(b.vel);
    }
}
