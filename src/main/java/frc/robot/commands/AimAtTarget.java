package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.config.ControllerConfig;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.launcher.LaunchCalculator;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.ShooterTarget;
import frc.robot.subsystems.launcher.TurretAngle;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.util.ControllerUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AimAtTarget extends Command {

    private static final DoubleWriter extraRPMWriter = Telemetry.makeDoubleWriterInitial(
            AimAtTarget.class.getSimpleName(), "extraRPM", TelemetryUnits.rpm, Double.NaN);

    private final Launcher launcher;
    private final Swerve swerve_noDep;
    private final Supplier<Translation2d> targetSupplier;
    private final BooleanSupplier overrideTurret;

    public AimAtTarget(
            Launcher _laucher,
            Swerve _swerve_noDep,
            Supplier<Translation2d> _targetSupplier,
            BooleanSupplier _overrideTurret) {
        launcher = _laucher;
        addRequirements(launcher);
        swerve_noDep = _swerve_noDep;

        targetSupplier = _targetSupplier;
        overrideTurret = _overrideTurret;
    }

    public static AimAtTarget atHub(
            Launcher launcher, Swerve swerve_noDep, boolean isRed, BooleanSupplier overrideTurret) {
        return new AimAtTarget(
                launcher,
                swerve_noDep,
                () -> (isRed ? FieldConstants.redHubLoc : FieldConstants.blueHubLoc),
                overrideTurret);
    }

    public static AimAtTarget atFZone(
            Launcher launcher, Swerve swerve_noDep, boolean isRed, BooleanSupplier overrideTurret) {
        return new AimAtTarget(
                launcher,
                swerve_noDep,
                () -> {
                    boolean inTop =
                            swerve_noDep.getStateCopy().Pose.getMeasureY().gt(FieldConstants.fieldYCenter);
                    if (isRed) {
                        return inTop ? FieldConstants.redZoneTopTargetLoc : FieldConstants.redZoneBottomTargetLoc;
                    } else {
                        return inTop ? FieldConstants.blueZoneTopTargetLoc : FieldConstants.blueZoneBottomTargetLoc;
                    }
                },
                overrideTurret);
    }

    @Override
    public void execute() {
        SwerveDriveState swerveState = swerve_noDep.getStateCopy();

        Pair<TurretAngle, ShooterTarget> target = LaunchCalculator.calcShot(
                targetSupplier.get(),
                swerveState.Pose,
                MetersPerSecond.of(swerveState.Speeds.vxMetersPerSecond),
                MetersPerSecond.of(swerveState.Speeds.vyMetersPerSecond));

        if (overrideTurret.getAsBoolean()) {
            launcher.setTurretDuty(-ControllerUtil.applyLinearDeadband(
                            RobotContainer.instance().driver2.getLeftX()
                                    + RobotContainer.instance().driver2.getRightX(),
                            ControllerConfig.overrideTurretDeadband)
                    * 0.1);
        } else {
            launcher.setTurretAngle(target.getFirst());
        }

        ShooterTarget st = target.getSecond();

        // quicky and hacky compensation button for driver 2
        double extraRPM = RobotContainer.instance().driver2.getRightBumperButton() ? 100 : 0;
        extraRPMWriter.set(extraRPM);
        st.add(ShooterTarget.fromShooterRPM(extraRPM));

        st.clampToLegalRange();
        launcher.setShooterTarget(st);
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopTurret();
        launcher.stopShooter();

        extraRPMWriter.set(Double.NaN);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
