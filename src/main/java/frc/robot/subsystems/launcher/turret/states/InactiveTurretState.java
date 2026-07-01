package frc.robot.subsystems.launcher.turret.states;

public record InactiveTurretState() implements ITurretState {
    @Override
    public boolean equals(Object other) {
        return other.getClass() == InactiveTurretState.class;
    }
}
