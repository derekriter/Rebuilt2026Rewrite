package frc.robot.auto;

import static frc.robot.auto.AutoCommands.*;
import static frc.robot.auto.generated.ChoreoTraj.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class Autos {

    private static void addAutoProgram(LoggedDashboardChooser<AutoProgram> chooser, AutoProgram prog) {
        chooser.addOption(prog.getName(), prog);
    }

    public static LoggedDashboardChooser<AutoProgram> createAutos(AutoFactory factory) {
        LoggedDashboardChooser<AutoProgram> chooser = new LoggedDashboardChooser<>("autoChooser");
        chooser.addDefaultOption("None", null);

        addAutoProgram(chooser, trench_pickup_shoot_climb(factory, true, false));
        addAutoProgram(chooser, trench_pickup_shoot_climb(factory, false, false));
        addAutoProgram(chooser, L_trench_pickup_depot(factory, false));
        addAutoProgram(chooser, trench_pickup_shoot_pickup_shoot(factory, true, false));
        addAutoProgram(chooser, trench_pickup_shoot_pickup_shoot(factory, false, false));

        return chooser;
    }

    private static AutoProgram trench_pickup_shoot_climb(AutoFactory factory, boolean isLeft, boolean isDebug) {
        String name = (isLeft ? "Left" : "Right") + " Trench, Pickup, Shoot, Climb";
        AutoRoutine routine = factory.newRoutine(name);

        AutoTrajectory trench_collect1 = L_trench_collect1.asAutoTraj(routine);
        AutoTrajectory collect1_shoot = (isLeft ? L_collect1_shoot : R_collect1_shoot).asAutoTraj(routine);
        AutoTrajectory shoot_climb = (isLeft ? L_shoot_climb : R_shoot_climb).asAutoTraj(routine);
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
                                    RobotContainer.instance().intakeCmd()
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

        return new AutoProgram(
                isDebug,
                name,
                isLeft ? SetupReference.LEFT_TRENCH : SetupReference.RIGHT_TRENCH,
                routine,
                trench_collect1,
                collect1_shoot,
                shoot_climb);
    }

    private static AutoProgram trench_pickup_shoot_pickup_shoot(AutoFactory factory, boolean isLeft, boolean isDebug) {
        String name = (isLeft ? "Left" : "Right") + " Trench, Pickup, Shoot, Pickup, Shoot";
        AutoRoutine routine = factory.newRoutine(name);

        AutoTrajectory trench_collect1 = routine.trajectory("L_trench_collect1");
        AutoTrajectory collect1_shoot = (isLeft ? L_collect1_shoot : R_collect1_shoot).asAutoTraj(routine);
        AutoTrajectory shoot_collect2 = (isLeft ? L_shoot_collect2 : R_shoot_collect2).asAutoTraj(routine);
        AutoTrajectory collect2_shoot = (isLeft ? L_collect2_shoot : R_collect2_shoot).asAutoTraj(routine);
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
                                    RobotContainer.instance().intakeCmd()
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
                                        RobotContainer.instance().intakeCmd()
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

        return new AutoProgram(
                isDebug,
                name,
                isLeft ? SetupReference.LEFT_TRENCH : SetupReference.RIGHT_TRENCH,
                routine,
                trench_collect1,
                collect1_shoot,
                shoot_collect2,
                collect2_shoot);
    }

    private static AutoProgram L_trench_pickup_depot(AutoFactory factory, boolean isDebug) {
        String name = "Left Trench, Pickup, Depot";
        AutoRoutine routine = factory.newRoutine(name);

        AutoTrajectory trench_collect1 = L_trench_collect1.asAutoTraj(routine);
        AutoTrajectory collect1_depot = L_collect1_depot.asAutoTraj(routine);

        // spotless:off
        routine.active()
                .onTrue(
                    Commands.sequence(
                            trench_collect1.resetOdometry(),
                            RobotContainer.instance().deployIntakeCmd(),
                            Commands.deadline(
                                    trench_collect1.cmd(),
                                    RobotContainer.instance().intakeCmd()
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

        return new AutoProgram(isDebug, name, SetupReference.LEFT_TRENCH, routine, trench_collect1, collect1_depot);
    }

    private Autos() {}
}
