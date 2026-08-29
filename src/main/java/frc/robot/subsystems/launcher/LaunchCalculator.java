package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.launcher.shooter.ShooterTarget;
import frc.robot.subsystems.launcher.turret.TurretAngle;

public final class LaunchCalculator {
    public static Pair<TurretAngle, ShooterTarget> calcShot(
            Translation2d target, Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {

        Translation2d launcherLoc = getLauncherLocOnField(robotPose);
        Translation2d compensatedTarget =
                target.plus(calcPointCompensation(robotRelativeSpeeds, robotPose.getRotation()));

        Translation2d vecToTarget = compensatedTarget.minus(launcherLoc);
        double fieldCentricDeg = Math.toDegrees(Math.atan2(vecToTarget.getY(), vecToTarget.getX()));
        double robotCentricDeg = fieldCentricDeg - robotPose.getRotation().getDegrees();

        double metersToTarget = vecToTarget.getNorm();

        return Pair.of(TurretAngle.fromMechanismDeg(robotCentricDeg), ShooterTarget.fromMetersToTarget(metersToTarget));
    }

    private static Translation2d getLauncherLocOnField(Pose2d robotPose) {
        return robotPose.getTranslation().plus(LauncherConstants.launcherOffset.rotateBy(robotPose.getRotation()));
    }

    private static Translation2d calcPointCompensation(ChassisSpeeds robotRelativeSpeeds, Rotation2d robotRot) {
        Translation2d velCompensation = new Translation2d(
                        robotRelativeSpeeds.vxMetersPerSecond * LauncherConstants.ballAirTime.in(Seconds),
                        robotRelativeSpeeds.vyMetersPerSecond * LauncherConstants.ballAirTime.in(Seconds))
                .rotateBy(robotRot)
                .unaryMinus();

        return velCompensation;
    }

    private LaunchCalculator() {}
}
