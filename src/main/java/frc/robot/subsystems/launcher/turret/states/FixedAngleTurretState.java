package frc.robot.subsystems.launcher.turret.states;

import frc.robot.subsystems.launcher.turret.TurretAngle;

public record FixedAngleTurretState(TurretAngle angle) implements ITurretState {
    @Override
    public boolean equals(Object other) {
        if (other.getClass() != FixedAngleTurretState.class) return false;

        return ((FixedAngleTurretState) other).angle.equals(other);
    }
}
