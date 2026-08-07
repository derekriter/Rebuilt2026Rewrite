package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.config.LauncherConfig.TurretConfig;

public final class TurretAngle {

    private MutAngle mechAngle;

    private TurretAngle(MutAngle _mechAngle) {
        mechAngle = _mechAngle;
    }

    public static TurretAngle fromMechanismAngle(Angle mechAngle) {
        return new TurretAngle(mechAngle.mutableCopy());
    }

    public static TurretAngle fromMechanismDeg(double mechDeg) {
        return new TurretAngle(Degrees.mutable(mechDeg));
    }

    public static TurretAngle fromMotorAngle(Angle motorAngle) {
        return new TurretAngle(motorAngle.mutableCopy().mut_divide(TurretConfig.motorRotsPerMechRots));
    }

    public static TurretAngle fromMotorRotations(double motorRots) {
        return new TurretAngle(Rotations.mutable(motorRots).mut_divide(TurretConfig.motorRotsPerMechRots));
    }

    public Angle asMechanismAngle() {
        return mechAngle.copy();
    }

    public Angle asMotorAngle() {
        return mechAngle.times(TurretConfig.motorRotsPerMechRots);
    }

    public double asMotorRotations() {
        return mechAngle.in(Rotations) * TurretConfig.motorRotsPerMechRots;
    }

    public boolean isWrapped() {
        return mechAngle.lt(TurretConfig.breakAngle.mechAngle)
                && mechAngle.gte(TurretConfig.breakAngle.mechAngle.minus(Rotations.of(1)));
    }

    public void wrap() {
        if (isWrapped()) return;

        double deg = mechAngle.in(Degrees);
        double breakDeg = TurretConfig.breakAngle.mechAngle.in(Degrees);

        // https://www.desmos.com/calculator/1wz1yp45dl
        mechAngle.mut_replace(deg - breakDeg - 360 * Math.floor((deg - breakDeg) / 360d) - 360 + breakDeg, Degrees);
    }

    public boolean isLegal() {
        return mechAngle.lte(TurretConfig.maxLegal.mechAngle) && mechAngle.gte(TurretConfig.minLegal.mechAngle);
    }

    public void clampToLegalRange() {
        if (isLegal()) return;

        wrap();
        double deg = mechAngle.in(Degrees);

        mechAngle.mut_replace(
                MathUtil.clamp(
                        deg, TurretConfig.minLegal.mechAngle.in(Degrees), TurretConfig.maxLegal.mechAngle.in(Degrees)),
                Degrees);
    }
}
