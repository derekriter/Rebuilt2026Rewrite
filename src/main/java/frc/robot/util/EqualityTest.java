package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;

@FunctionalInterface
public interface EqualityTest<T> {
    /**
     * Tests if the two given values are equal and returns {@code true} if so
     */
    boolean test(T a, T b);

    public static final EqualityTest<Pose2d> pose2dEqualityTest = (Pose2d a, Pose2d b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };

    public static final EqualityTest<Pose2d[]> pose2dArrayEqualityTest = (Pose2d[] a, Pose2d[] b) -> {
        return Arrays.equals(a, b);
    };

    public static final EqualityTest<Pose3d> pose3dEqualityTest = (Pose3d a, Pose3d b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };

    public static final EqualityTest<Pose3d[]> pose3dArrayEqualityTest = (Pose3d[] a, Pose3d[] b) -> {
        return Arrays.equals(a, b);
    };

    public static final EqualityTest<ChassisSpeeds> chassisSpeedsEqualityTest = (ChassisSpeeds a, ChassisSpeeds b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };

    public static final EqualityTest<SwerveModuleState> swerveModuleStateEqualityTest =
            (SwerveModuleState a, SwerveModuleState b) -> {
                if (a == null) return b == null;
                return a.equals(b);
            };

    public static final EqualityTest<SwerveModuleState[]> swerveModuleStateArrayEqualityTest =
            (SwerveModuleState[] a, SwerveModuleState[] b) -> {
                return Arrays.equals(a, b);
            };

    public static final EqualityTest<SwerveModulePosition> swerveModulePositionEqualityTest =
            (SwerveModulePosition a, SwerveModulePosition b) -> {
                if (a == null) return b == null;
                return a.equals(b);
            };

    public static final EqualityTest<SwerveModulePosition[]> swerveModulePositionArrayEqualityTest =
            (SwerveModulePosition[] a, SwerveModulePosition[] b) -> {
                return Arrays.equals(a, b);
            };
}
