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
import edu.wpi.first.math.trajectory.Trajectory;

public final class BlankValues {

    public static final Rotation2d rotation2d = new Rotation2d(Double.NaN);
    public static final Rotation2d[] rotation2dArray = new Rotation2d[0];

    public static final Translation2d translation2d = new Translation2d(Double.NaN, Double.NaN);
    public static final Translation2d[] translation2dArray = new Translation2d[0];

    public static final Pose2d pose2d = new Pose2d(Double.NaN, Double.NaN, rotation2d);
    public static final Pose2d[] pose2dArray = new Pose2d[0];

    public static final Rotation3d rotation3d = new Rotation3d(Double.NaN, Double.NaN, Double.NaN);
    public static final Rotation3d[] rotation3dArray = new Rotation3d[0];

    public static final Translation3d translation3d = new Translation3d(Double.NaN, Double.NaN, Double.NaN);
    public static final Translation3d[] translation3dArray = new Translation3d[0];

    public static final Pose3d pose3d = new Pose3d(Double.NaN, Double.NaN, Double.NaN, rotation3d);
    public static final Pose3d[] pose3dArray = new Pose3d[0];

    public static final ChassisSpeeds chassisSpeeds = new ChassisSpeeds(Double.NaN, Double.NaN, Double.NaN);

    public static final SwerveModuleState swerveModuleState = new SwerveModuleState(Double.NaN, rotation2d);
    public static final SwerveModuleState[] swerveModuleStateArray = new SwerveModuleState[0];

    public static final SwerveModulePosition swerveModulePosition = new SwerveModulePosition(Double.NaN, rotation2d);
    public static final SwerveModulePosition[] swerveModulePositionArray = new SwerveModulePosition[0];

    public static final String string = "null";

    public static final Trajectory trajectory = new Trajectory();

    private BlankValues() {}
}
