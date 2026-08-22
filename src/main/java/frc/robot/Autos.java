package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.telemetry.Telemetry;

public final class Autos {

    public static AutoChooser createAutos(AutoFactory factory) {
        AutoChooser chooser = new AutoChooser();

        chooser.addCmd("Left Trench, Pickup, Shoot, Climb", () -> trench_pickup_shoot_climb(factory, true));
        chooser.addCmd("Right Trench, Pickup, Shoot, Climb", () -> trench_pickup_shoot_climb(factory, false));

        return chooser;
    }

    private static Command waitForReadyToShootCmd(double timeout_s) {
        return Commands.waitUntil(() -> Robot.instance().brain.state.launcherReport.shooterIsAtTarget
                        && Robot.instance().brain.state.launcherReport.turretIsAtTarget)
                .withTimeout(timeout_s)
                .withName("waitForReadyToShootCmd");
    }

    private static Command waitUntilInFZoneCmd() {
        return Commands.waitUntil(() -> Robot.instance().brain.isInFZone()).withName("waitUntilInFZoneCmd");
    }

    private static Command shootCmd(double timeout_s) {
        return Commands.runOnce(() -> Telemetry.println("shoot"))
                .withTimeout(timeout_s)
                .withName("shootCmd");
    }

    private static Command alignWithTowerCmd() {
        ApplyRobotSpeeds towardsReq = new ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSpeeds(ChassisSpeeds.discretize(new ChassisSpeeds(0, -0.5, 0), 0.02));
        ApplyRobotSpeeds finalAlignReq = new ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSpeeds(ChassisSpeeds.discretize(new ChassisSpeeds(-0.5, 0, 0), 0.02));

        return Commands.sequence(
                        Commands.runOnce(
                                () -> RobotContainer.instance().swerve.setControl(towardsReq),
                                RobotContainer.instance().swerve),
                        Commands.waitUntil(() -> /*RobotContainer.instance().climb.canSeeTower()*/ true)
                                .withTimeout(5),
                        Commands.runEnd(
                                        () -> RobotContainer.instance().swerve.setControl(finalAlignReq),
                                        () -> RobotContainer.instance().swerve.stop(),
                                        RobotContainer.instance().swerve)
                                .withTimeout(0.25))
                .withName("alignWithTowerCmd");
    }

    private static Command trench_pickup_shoot_climb(AutoFactory factory, boolean isLeft) {
        AutoRoutine routine = factory.newRoutine((isLeft ? "Left" : "Right") + " Trench, Pickup, Shoot, Climb");

        String prefix = isLeft ? "L_" : "R_";
        AutoTrajectory trench_collect1 = routine.trajectory("L_trench_collect1");
        AutoTrajectory collect1_shoot = routine.trajectory(prefix + "collect1_shoot");
        AutoTrajectory shoot_climb = routine.trajectory(prefix + "shoot_climb");
        if (!isLeft) {
            trench_collect1 = trench_collect1.mirrorY();
        }

        // spotless:off
        routine.active()
                .onTrue(
                    Commands.sequence(
                            trench_collect1.resetOdometry(),
                            RobotContainer.instance().deployIntakeCmd(),
                            Commands.deadline(
                                    trench_collect1.cmd(),
                                    RobotContainer.instance().runIntakeCmd()
                            ),
                            Commands.parallel(
                                    Commands.sequence(
                                            Commands.parallel(
                                                    collect1_shoot.cmd(),
                                                    Commands.sequence(
                                                            waitUntilInFZoneCmd(),
                                                            waitForReadyToShootCmd(4),
                                                            shootCmd(4)
                                                    )
                                            ),
                                            shoot_climb.cmd()
                                    ),
                                    Commands.sequence(
                                            waitUntilInFZoneCmd(),
                                            RobotContainer.instance().climbUpPosCmd()
                                    )
                            ),
                            alignWithTowerCmd(),
                            RobotContainer.instance().climbHangingPosCmd()
                    )
                );
        //spotless:on

        return routine.cmd();
    }

    private Autos() {}
}
