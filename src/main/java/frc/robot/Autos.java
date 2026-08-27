package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Console;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class Autos {

    public static LoggedDashboardChooser<Command> createAutos(AutoFactory factory) {
        LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("autoChooser");
        chooser.addDefaultOption("None", null);

        chooser.addOption("Left Trench, Pickup, Shoot, Climb", trench_pickup_shoot_climb(factory, true));
        chooser.addOption("Right Trench, Pickup, Shoot, Climb", trench_pickup_shoot_climb(factory, false));
        chooser.addOption("Left Trench, Pickup, Depot", L_trench_pickup_depot(factory));
        chooser.addOption("Left Trench, Pickup, Shoot, Pickup, Shoot", trench_pickup_shoot_pickup_shoot(factory, true));
        chooser.addOption(
                "Right Trench, Pickup, Shoot, Pickup, Shoot", trench_pickup_shoot_pickup_shoot(factory, false));

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
        return shootCmd().withTimeout(timeout_s);
    }

    private static Command shootCmd() {
        return Commands.startRun(() -> Console.println("shoot"), () -> {}).withName("shootCmd");
    }

    private static Command alignWithTowerCmd() {
        return Commands.sequence(
                        Commands.runOnce(
                                () -> RobotContainer.instance()
                                        .swerve
                                        .runRobotRelativeVelocity(new ChassisSpeeds(0, -0.5, 0)),
                                RobotContainer.instance().swerve),
                        Commands.waitUntil(() -> /*RobotContainer.instance().climb.canSeeTower()*/ true)
                                .withTimeout(5),
                        Commands.runEnd(
                                        () -> RobotContainer.instance()
                                                .swerve
                                                .runRobotRelativeVelocity(new ChassisSpeeds(-0.5, 0, 0)),
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

    private static Command trench_pickup_shoot_pickup_shoot(AutoFactory factory, boolean isLeft) {
        AutoRoutine routine = factory.newRoutine((isLeft ? "Left" : "Right") + " Trench, Pickup, Shoot, Pickup, Shoot");

        String prefix = isLeft ? "L_" : "R_";
        AutoTrajectory trench_collect1 = routine.trajectory("L_trench_collect1");
        AutoTrajectory collect1_shoot = routine.trajectory(prefix + "collect1_shoot");
        AutoTrajectory shoot_collect2 = routine.trajectory(prefix + "shoot_collect2");
        AutoTrajectory collect2_shoot = routine.trajectory(prefix + "collect2_shoot");
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
                                    collect1_shoot.cmd(),
                                    Commands.sequence(
                                            waitUntilInFZoneCmd(),
                                            waitForReadyToShootCmd(2),
                                            shootCmd(4)
                                    )
                            ),
                            Commands.deadline(
                                        shoot_collect2.cmd(),
                                        RobotContainer.instance().runIntakeCmd()
                            ),
                            Commands.parallel(
                                    collect2_shoot.cmd(),
                                    Commands.sequence(
                                            waitUntilInFZoneCmd(),
                                            waitForReadyToShootCmd(2),
                                            shootCmd()
                                    )
                            )
                    )
                );
        //spotless:on

        return routine.cmd();
    }

    private static Command L_trench_pickup_depot(AutoFactory factory) {
        AutoRoutine routine = factory.newRoutine("Left Trench, Pickup, Depot");

        AutoTrajectory trench_collect1 = routine.trajectory("L_trench_collect1");
        AutoTrajectory collect1_depot = routine.trajectory("L_collect1_depot");

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
                                    collect1_depot.cmd(),
                                    Commands.sequence(
                                            waitUntilInFZoneCmd(),
                                            waitForReadyToShootCmd(4),
                                            shootCmd()
                                    )
                            )
                    )
                );
        //spotless:on

        return routine.cmd();
    }

    private Autos() {}
}
