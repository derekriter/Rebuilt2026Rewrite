package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.Console;

public final class AutoCommands {

    public static Command waitForReadyToShootCmd(double timeout_s) {
        return Commands.waitUntil(() -> Robot.instance().brain.state.launcherReport.shooterIsAtTarget
                        && Robot.instance().brain.state.launcherReport.turretIsAtTarget)
                .withTimeout(timeout_s)
                .withName("waitForReadyToShootCmd");
    }

    public static Command waitUntilInFZoneCmd() {
        return Commands.waitUntil(() -> Robot.instance().brain.isInFZone()).withName("waitUntilInFZoneCmd");
    }

    public static Command shootCmd(double timeout_s) {
        return shootCmd().withTimeout(timeout_s);
    }

    public static Command shootCmd() {
        return Commands.startRun(() -> Console.println("shoot"), () -> {}).withName("shootCmd");
    }

    public static Command alignWithTowerCmd() {
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

    private AutoCommands() {}
}
