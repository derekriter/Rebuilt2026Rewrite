package frc.robot.subsystems.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.config.LauncherConstants.TurretConstants;

public final class TurretAngle {

    private final Angle mechAngle;

    private TurretAngle(Angle _mechAngle) {
        mechAngle = _mechAngle;
    }

    public static TurretAngle fromMechanismAngle(Angle mechAngle) {
        return new TurretAngle(mechAngle);
    }

    public static TurretAngle fromMotorAngle(Angle motorAngle) {
        return fromMechanismAngle(motorAngle.div(TurretConstants.motorRotsPerMechRots));
    }

    public Angle asMechanismAngle() {
        return mechAngle;
    }

    public Angle asMotorAngle() {
        return mechAngle.times(TurretConstants.motorRotsPerMechRots);
    }

    public boolean isWrapped() {
        return mechAngle.lt(TurretConstants.breakAngle.asMechanismAngle())
                && mechAngle.gte(TurretConstants.breakAngle.asMechanismAngle().minus(Rotations.of(1)));
    }

    public TurretAngle wrap() {
        double deg = asMechanismAngle().in(Degrees);
        double breakDeg = TurretConstants.breakAngle.asMechanismAngle().in(Degrees);

        // https://www.desmos.com/calculator/1wz1yp45dl
        return fromMechanismAngle(
                Degrees.of(deg - breakDeg - 360 * Math.floor((deg - breakDeg) / 360d) - 360 + breakDeg));
    }

    public TurretAngle clampToLegalRange() {
        double deg = (isWrapped() ? this : wrap()).asMechanismAngle().in(Degrees);

        return fromMechanismAngle(Degrees.of(MathUtil.clamp(
                deg,
                TurretConstants.minLegal.asMechanismAngle().in(Degrees),
                TurretConstants.maxLegal.asMechanismAngle().in(Degrees))));
    }

    public boolean isLegal() {
        return mechAngle.lte(TurretConstants.maxLegal.asMechanismAngle())
                && mechAngle.gte(TurretConstants.minLegal.asMechanismAngle());
    }

    @Override
    public boolean equals(Object other) {
        if (other.getClass() != TurretAngle.class) return false;

        return ((TurretAngle) other).mechAngle.isEquivalent(mechAngle);
    }
}
