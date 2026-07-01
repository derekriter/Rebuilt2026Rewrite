package frc.robot.subsystems.launcher.shooter.states;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;

public record TargetingPointShooterState(Supplier<Translation2d> point) implements IShooterState {
    @Override
    public boolean equals(Object other) {
        if (other.getClass() != TargetingPointShooterState.class) return false;

        return ((TargetingPointShooterState) other).point.equals(point);
    }
}
