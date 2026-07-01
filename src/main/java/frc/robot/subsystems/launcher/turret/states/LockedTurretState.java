package frc.robot.subsystems.launcher.turret.states;

public record LockedTurretState() implements ITurretState {
    @Override
    public boolean equals(Object other) {
        return other.getClass() == LockedTurretState.class;
    }
}
