package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@FunctionalInterface
public interface CloneOperation<T> {
    /**
     * Return an object of type {@code T} that is nearly equal if not equal to {@code original}
     */
    T clone(T original);

    public static final CloneOperation<Pose2d> pose2dCloneOperation = (Pose2d original) -> {
        return original; // safe to return original because Pose2ds are immutable
    };

    public static final CloneOperation<Pose2d[]> pose2dArrayCloneOperation = (Pose2d[] original) -> {
        if (original == null) return null;
        return original.clone();
    };

    public static final CloneOperation<Pose3d> pose3dCloneOperation = (Pose3d original) -> {
        return original; // safe to return original because Pose3ds are immutable
    };

    public static final CloneOperation<Pose3d[]> pose3dArrayCloneOperation = (Pose3d[] original) -> {
        if (original == null) return null;
        return original.clone();
    };

    public static final CloneOperation<ChassisSpeeds> chassisSpeedsCloneOperation = (ChassisSpeeds original) -> {
        if (original == null) return null;
        return new ChassisSpeeds(
                original.vxMetersPerSecond, original.vyMetersPerSecond, original.omegaRadiansPerSecond);
    };

    public static final CloneOperation<SwerveModuleState> swerveModuleStateCloneOperation =
            (SwerveModuleState original) -> {
                if (original == null) return null;
                return new SwerveModuleState(original.speedMetersPerSecond, original.angle);
            };

    public static final CloneOperation<SwerveModuleState[]> swerveModuleStateArrayCloneOperation =
            (SwerveModuleState[] original) -> {
                if (original == null) return null;

                SwerveModuleState[] clone = new SwerveModuleState[original.length];
                for (int i = 0; i < original.length; i++) {
                    clone[i] = swerveModuleStateCloneOperation.clone(original[i]);
                }
                return clone;
            };

    public static final CloneOperation<SwerveModulePosition> swerveModulePositionCloneOperation =
            (SwerveModulePosition original) -> {
                if (original == null) return null;
                return original.copy();
            };

    public static final CloneOperation<SwerveModulePosition[]> swerveModulePositionArrayCloneOperation =
            (SwerveModulePosition[] original) -> {
                if (original == null) return null;

                SwerveModulePosition[] clone = new SwerveModulePosition[original.length];
                for (int i = 0; i < original.length; i++) {
                    clone[i] = swerveModulePositionCloneOperation.clone(original[i]);
                }
                return clone;
            };
}
