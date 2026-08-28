// spotless:off
package frc.robot.auto.generated;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final LinearVelocity collectVel = Units.MetersPerSecond.of(2);
    public static final LinearVelocity depotVel = Units.MetersPerSecond.of(0.5);
    public static final Distance fieldHeight = Units.Meters.of(8.069326);
    public static final Distance trenchTolerance = Units.Meters.of(0.099098);

    public static final class Poses {
        public static final Pose2d climb_L = new Pose2d(1.03, 4.8, Rotation2d.fromRadians(0));
        public static final Pose2d climb_R = new Pose2d(1.08, 2.691, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d collect1_L = new Pose2d(7.67, 4, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d collect1_R = new Pose2d(7.67, 4.069326, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d collect2_L = new Pose2d(6.281, 3, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d collect2_R = new Pose2d(6.281, 5.069326, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d depot = new Pose2d(0.614014, 5.9605227, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d shoot_L = new Pose2d(2.325, 6.144, Rotation2d.fromRadians(-1.0471976));
        public static final Pose2d shoot_R = new Pose2d(2.325, 1.925326, Rotation2d.fromRadians(0));
        public static final Pose2d trenchFEnd_L = new Pose2d(3.6139543, 7.4519553, Rotation2d.fromRadians(0));
        public static final Pose2d trenchFEnd_R = new Pose2d(3.6139543, 0.6173707, Rotation2d.fromRadians(0));
        public static final Pose2d trenchNEnd_L = new Pose2d(5.4486299, 7.473434, Rotation2d.fromRadians(0));
        public static final Pose2d trenchNEnd_R = new Pose2d(5.4486299, 0.595892, Rotation2d.fromRadians(0));
        public static final Pose2d trench_L = new Pose2d(4.35, 7.63, Rotation2d.fromRadians(0));
    }
}
// spotless:on
