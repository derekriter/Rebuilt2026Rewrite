package frc.robot.subsystems.launcher.turret.states;

public record ManualControlTurretState(double duty) implements ITurretState {
    @Override
    public boolean equals(Object other) {
        if (other.getClass() != ManualControlTurretState.class) return false;

        return ((ManualControlTurretState) other).duty == duty;
    }
}
