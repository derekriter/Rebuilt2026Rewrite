package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static final EqualityTest<Rotation2d> rotation2d = (Rotation2d a, Rotation2d b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };
    public static final EqualityTest<Rotation2d[]> rotation2dArray = (Rotation2d[] a, Rotation2d[] b) -> {
        return Arrays.equals(a, b);
    };

    public static final EqualityTest<Translation2d> translation2d = (Translation2d a, Translation2d b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };
    public static final EqualityTest<Translation2d[]> translation2dArray = (Translation2d[] a, Translation2d[] b) -> {
        return Arrays.equals(a, b);
    };

    public static final EqualityTest<Pose2d> pose2d = (Pose2d a, Pose2d b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };
    public static final EqualityTest<Pose2d[]> pose2dArray = (Pose2d[] a, Pose2d[] b) -> {
        return Arrays.equals(a, b);
    };

    public static final EqualityTest<Rotation3d> rotation3d = (Rotation3d a, Rotation3d b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };
    public static final EqualityTest<Rotation3d[]> rotation3dArray = (Rotation3d[] a, Rotation3d[] b) -> {
        return Arrays.equals(a, b);
    };

    public static final EqualityTest<Translation3d> translation3d = (Translation3d a, Translation3d b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };
    public static final EqualityTest<Translation3d[]> translation3dArray = (Translation3d[] a, Translation3d[] b) -> {
        return Arrays.equals(a, b);
    };

    public static final EqualityTest<Pose3d> pose3d = (Pose3d a, Pose3d b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };
    public static final EqualityTest<Pose3d[]> pose3dArray = (Pose3d[] a, Pose3d[] b) -> {
        return Arrays.equals(a, b);
    };

    public static final EqualityTest<ChassisSpeeds> chassisSpeeds = (ChassisSpeeds a, ChassisSpeeds b) -> {
        if (a == null) return b == null;
        return a.equals(b);
    };

    public static final EqualityTest<SwerveModuleState> swerveModuleState =
            (SwerveModuleState a, SwerveModuleState b) -> {
                if (a == null) return b == null;
                return a.equals(b);
            };
    public static final EqualityTest<SwerveModuleState[]> swerveModuleStateArray =
            (SwerveModuleState[] a, SwerveModuleState[] b) -> {
                return Arrays.equals(a, b);
            };

    public static final EqualityTest<SwerveModulePosition> swerveModulePosition =
            (SwerveModulePosition a, SwerveModulePosition b) -> {
                if (a == null) return b == null;
                return a.equals(b);
            };
    public static final EqualityTest<SwerveModulePosition[]> swerveModulePositionArray =
            (SwerveModulePosition[] a, SwerveModulePosition[] b) -> {
                return Arrays.equals(a, b);
            };
}
