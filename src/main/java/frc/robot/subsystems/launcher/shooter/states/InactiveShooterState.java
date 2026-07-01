package frc.robot.subsystems.launcher.shooter.states;

public record InactiveShooterState() implements IShooterState {
    @Override
    public boolean equals(Object other) {
        return other.getClass() == InactiveShooterState.class;
    }
}
