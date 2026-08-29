package frc.robot.subsystems.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.constants.LauncherConstants.TurretConstants;

public final class TurretAngle {

    private MutAngle mechAngle;

    private TurretAngle(MutAngle _mechAngle) {
        mechAngle = _mechAngle;
    }

    // from
    public static TurretAngle fromMechanismAngle(Angle mechAngle) {
        return new TurretAngle(mechAngle.mutableCopy());
    }

    public static TurretAngle fromMechanismDeg(double mechDeg) {
        return new TurretAngle(Degrees.mutable(mechDeg));
    }

    public static TurretAngle fromMotorAngle(Angle motorAngle) {
        return new TurretAngle(Rotations.mutable(motorAngleToMechAngle_ul(motorAngle.in(Rotations))));
    }

    public static TurretAngle fromMotorRotations(double motorRots) {
        return new TurretAngle(Rotations.mutable(motorAngleToMechAngle_ul(motorRots)));
    }

    // as
    public Angle asMechanismAngle() {
        return mechAngle.copy();
    }

    public double asMechanismDegrees() {
        return mechAngle.in(Degrees);
    }

    public Angle asMotorAngle() {
        return Rotations.of(mechAngleToMotorAngle_ul(mechAngle.in(Rotations)));
    }

    public double asMotorRotations() {
        return mechAngleToMotorAngle_ul(mechAngle.in(Rotations));
    }

    // conversions
    public static Angle mechAngleToMotorAngle(Angle mechAngle) {
        return mechAngle.div(TurretConstants.mechRotsPerMotorRots);
    }

    public static double mechAngleToMotorAngle_ul(double mechAngle_ul) {
        return mechAngle_ul / TurretConstants.mechRotsPerMotorRots;
    }

    public static Angle motorAngleToMechAngle(Angle motorAngle) {
        return motorAngle.times(TurretConstants.mechRotsPerMotorRots);
    }

    public static double motorAngleToMechAngle_ul(double motorAngle_ul) {
        return motorAngle_ul * TurretConstants.mechRotsPerMotorRots;
    }

    // tests + operators
    public boolean isWrapped() {
        return mechAngle.lt(TurretConstants.breakAngle.mechAngle)
                && mechAngle.gte(TurretConstants.breakAngle.mechAngle.minus(Rotations.of(1)));
    }

    public void wrap() {
        if (isWrapped()) return;

        double deg = mechAngle.in(Degrees);
        double breakDeg = TurretConstants.breakAngle.mechAngle.in(Degrees);

        // https://www.desmos.com/calculator/1wz1yp45dl
        mechAngle.mut_replace(deg - breakDeg - 360 * Math.floor((deg - breakDeg) / 360d) - 360 + breakDeg, Degrees);
    }

    public boolean isLegal() {
        return mechAngle.lte(TurretConstants.maxLegal.mechAngle) && mechAngle.gte(TurretConstants.minLegal.mechAngle);
    }

    public void clampToLegalRange() {
        if (isLegal()) return;

        wrap();
        double deg = mechAngle.in(Degrees);

        mechAngle.mut_replace(
                MathUtil.clamp(
                        deg,
                        TurretConstants.minLegal.mechAngle.in(Degrees),
                        TurretConstants.maxLegal.mechAngle.in(Degrees)),
                Degrees);
    }
}
