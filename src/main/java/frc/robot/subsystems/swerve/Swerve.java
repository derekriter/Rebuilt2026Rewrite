package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    public Pose2d getPose() {
        return Pose2d.kZero;
    }

    public Pair<LinearVelocity, LinearVelocity> getRealFieldRelativeVelocity() {
        return new Pair<LinearVelocity, LinearVelocity>(MetersPerSecond.zero(), MetersPerSecond.zero());
    }
}
