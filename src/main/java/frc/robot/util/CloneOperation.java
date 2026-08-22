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

@FunctionalInterface
public interface CloneOperation<T> {
    T clone(T original);

    public static final CloneOperation<Rotation2d> rotation2d = (Rotation2d original) -> {
        return original; // safe to return original because Rotation2ds are immutable
    };
    public static final CloneOperation<Rotation2d[]> rotation2dArray = (Rotation2d[] original) -> {
        if (original == null) return null;
        return original.clone();
    };

    public static final CloneOperation<Translation2d> translation2d = (Translation2d original) -> {
        return original; // safe to return original because Translation2ds are immutable
    };
    public static final CloneOperation<Translation2d[]> translation2dArray = (Translation2d[] original) -> {
        if (original == null) return null;
        return original.clone();
    };

    public static final CloneOperation<Pose2d> pose2d = (Pose2d original) -> {
        return original; // safe to return original because Pose2ds are immutable
    };
    public static final CloneOperation<Pose2d[]> pose2dArray = (Pose2d[] original) -> {
        if (original == null) return null;
        return original.clone();
    };

    public static final CloneOperation<Rotation3d> rotation3d = (Rotation3d original) -> {
        return original; // safe to return original because Rotation3ds are immutable
    };
    public static final CloneOperation<Rotation3d[]> rotation3dArray = (Rotation3d[] original) -> {
        if (original == null) return null;
        return original.clone();
    };

    public static final CloneOperation<Translation3d> translation3d = (Translation3d original) -> {
        return original; // safe to return original because Translation3ds are immutable
    };
    public static final CloneOperation<Translation3d[]> translation3dArray = (Translation3d[] original) -> {
        if (original == null) return null;
        return original.clone();
    };

    public static final CloneOperation<Pose3d> pose3d = (Pose3d original) -> {
        return original; // safe to return original because Pose3ds are immutable
    };
    public static final CloneOperation<Pose3d[]> pose3dArray = (Pose3d[] original) -> {
        if (original == null) return null;
        return original.clone();
    };

    public static final CloneOperation<ChassisSpeeds> chassisSpeeds = (ChassisSpeeds original) -> {
        if (original == null) return null;
        return new ChassisSpeeds(
                original.vxMetersPerSecond, original.vyMetersPerSecond, original.omegaRadiansPerSecond);
    };

    public static final CloneOperation<SwerveModuleState> swerveModuleState = (SwerveModuleState original) -> {
        if (original == null) return null;
        return new SwerveModuleState(original.speedMetersPerSecond, original.angle);
    };
    public static final CloneOperation<SwerveModuleState[]> swerveModuleStateArray = (SwerveModuleState[] original) -> {
        if (original == null) return null;

        SwerveModuleState[] clone = new SwerveModuleState[original.length];
        for (int i = 0; i < original.length; i++) {
            clone[i] = swerveModuleState.clone(original[i]);
        }
        return clone;
    };

    public static final CloneOperation<SwerveModulePosition> swerveModulePosition = (SwerveModulePosition original) -> {
        if (original == null) return null;
        return original.copy();
    };
    public static final CloneOperation<SwerveModulePosition[]> swerveModulePositionArray =
            (SwerveModulePosition[] original) -> {
                if (original == null) return null;

                SwerveModulePosition[] clone = new SwerveModulePosition[original.length];
                for (int i = 0; i < original.length; i++) {
                    clone[i] = swerveModulePosition.clone(original[i]);
                }
                return clone;
            };
}
