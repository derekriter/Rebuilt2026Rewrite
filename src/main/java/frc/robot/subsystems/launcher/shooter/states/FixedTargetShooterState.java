package frc.robot.subsystems.launcher.shooter.states;

import frc.robot.subsystems.launcher.shooter.ShooterTarget;

public record FixedTargetShooterState(ShooterTarget target) implements IShooterState {
    @Override
    public boolean equals(Object other) {
        if (other.getClass() != FixedTargetShooterState.class) return false;

        return ((FixedTargetShooterState) other).target.equals(target);
    }
}
