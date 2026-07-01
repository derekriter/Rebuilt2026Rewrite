package frc.robot.subsystems.launcher.turret.states;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;

public record TargetPointTurretState(Supplier<Translation2d> point) implements ITurretState {
    @Override
    public boolean equals(Object other) {
        if (other.getClass() != TargetPointTurretState.class) return false;

        return ((TargetPointTurretState) other).point.equals(point);
    }
}
