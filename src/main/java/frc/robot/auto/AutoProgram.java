package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.util.BlankValues;
import frc.robot.util.Console;
import java.util.ArrayList;
import java.util.List;

public class AutoProgram {

    public static void clearPathDisplay() {
        try {
            Robot.instance().field.getObject("traj").setTrajectory(BlankValues.trajectory);
        } catch (Throwable e) {
            Console.reportError(e, true);
        }
    }

    private static Trajectory getWPITrajFromChoreoAutoTraj(AutoTrajectory autoTraj) {
        try {
            // kinda sketchy since getRawTrajectory is type unsafe
            List<Pose2d> samples = autoTraj.getRawTrajectory().samples().stream()
                    .map(s -> s.getPose())
                    .toList();

            if (samples.size() < 2) {
                return BlankValues.trajectory;
            }

            List<Translation2d> interiorPoints = new ArrayList<>(samples.size() - 2);
            for (int i = 1; i < samples.size() - 1; i++) {
                interiorPoints.add(samples.get(i).getTranslation());
            }

            return TrajectoryGenerator.generateTrajectory(
                    samples.get(0), interiorPoints, samples.get(samples.size() - 1), new TrajectoryConfig(100, 100));
        } catch (Throwable e) {
            Console.reportError(e, true);
            return BlankValues.trajectory;
        }
    }

    private static Trajectory flipWPITrajectoryToRed(Trajectory wpiTraj) {
        return new Trajectory(wpiTraj.getStates().stream()
                .map(s -> new Trajectory.State(
                        s.timeSeconds,
                        s.velocityMetersPerSecond,
                        s.accelerationMetersPerSecondSq,
                        s.poseMeters.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg),
                        s.curvatureRadPerMeter))
                .toList());
    }

    private final String name;
    private final boolean isDebug;
    private final Command cmd;
    private final Trajectory path_nl;
    private final Trajectory flipped_path_nl;

    public AutoProgram(boolean _isDebug, String _name, Command _cmd, Trajectory _path_nl) {
        isDebug = _isDebug;
        if (isDebug) {
            name = "[DEBUG] " + _name;
        } else {
            name = _name;
        }
        cmd = _cmd.withName(name);

        path_nl = _path_nl;
        if (path_nl == null) {
            flipped_path_nl = null;
        } else {
            flipped_path_nl = flipWPITrajectoryToRed(_path_nl);
        }
    }

    public AutoProgram(boolean _isDebug, String _name, AutoRoutine routine, AutoTrajectory... trajs) {
        isDebug = _isDebug;
        if (isDebug) {
            name = "[DEBUG] " + _name;
        } else {
            name = _name;
        }
        cmd = routine.cmd().withName(name);

        Trajectory workingPath_nl = null;
        for (AutoTrajectory t : trajs) {
            Trajectory wpiTraj = getWPITrajFromChoreoAutoTraj(t);
            if (workingPath_nl == null) {
                workingPath_nl = wpiTraj;
            } else {
                workingPath_nl = workingPath_nl.concatenate(wpiTraj);
            }
        }
        path_nl = workingPath_nl;

        if (path_nl == null) {
            flipped_path_nl = null;
        } else {
            flipped_path_nl = flipWPITrajectoryToRed(path_nl);
        }
    }

    public String getName() {
        return name;
    }

    public boolean getIsDebug() {
        return isDebug;
    }

    public Command getCommand() {
        return cmd;
    }

    public void displayPath() {
        Trajectory chosenPath_nl = Robot.instance().brain.state.isRed ? flipped_path_nl : path_nl;

        if (chosenPath_nl == null) {
            clearPathDisplay();
        } else {
            Robot.instance().field.getObject("traj").setTrajectory(chosenPath_nl);
        }
    }
}
