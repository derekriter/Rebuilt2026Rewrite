package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.config.ControllerConfig;
import frc.robot.config.SwerveConfig;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ControllerUtil;
import org.littletonrobotics.junction.Logger;

public class TeleopDrive extends Command {

    private final Swerve swerve;
    private final SwerveDriveBrake brake = new SwerveDriveBrake();
    private final RobotCentric robotOmega = new RobotCentric();
    // .withDeadband(SwerveConfig.deadbandTranslationVel)
    // .withRotationalDeadband(SwerveConfig.deadbandAngularVel);
    private final RobotCentricFacingAngle robotPOV = new RobotCentricFacingAngle()
            // .withDeadband(SwerveConfig.deadbandTranslationVel)
            // .withRotationalDeadband(SwerveConfig.deadbandAngularVel)
            .withHeadingPID(SwerveConfig.headingP, SwerveConfig.headingI, SwerveConfig.headingD);
    private final FieldCentric fieldOmega = new FieldCentric();
    // .withDeadband(SwerveConfig.deadbandTranslationVel)
    // .withRotationalDeadband(SwerveConfig.deadbandAngularVel);
    private final FieldCentricFacingAngle fieldPOV = new FieldCentricFacingAngle()
            // .withDeadband(SwerveConfig.deadbandTranslationVel)
            // .withRotationalDeadband(SwerveConfig.deadbandAngularVel)
            .withHeadingPID(SwerveConfig.headingP, SwerveConfig.headingI, SwerveConfig.headingD);

    private final SlewRateLimiter xSlew =
            new SlewRateLimiter(SwerveConfig.teleopTranslationAcceleration.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter ySlew =
            new SlewRateLimiter(SwerveConfig.teleopTranslationAcceleration.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter omegaSlew =
            new SlewRateLimiter(SwerveConfig.teleopAngularAcceleration.in(RadiansPerSecondPerSecond));

    public TeleopDrive(Swerve _swerve) {
        swerve = _swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        XboxController driver1 = RobotContainer.instance().driver1;
        // XboxController driver2 = RobotContainer.instance().driver2;

        double speedShifter =
                ControllerUtil.isPastDeadband(driver1.getRightTriggerAxis(), ControllerConfig.triggerThreshold)
                        ? 0.25
                        : 1;
        Logger.recordOutput("TeleopDrive/speedShifter", speedShifter);
        double slowDown;
        // if (driver1.getRightBumperButton()) {
        //     slowDown = 1;
        // } else if (driver2.getRightTriggerAxis() >= 0.5) {
        //     slowDown = 0.25 * 0.5;
        // } else if (driver2.getRightTriggerAxis() >= 0.5) {
        //     slowDown = 0.5;
        // } else {
        //     slowDown = 1;
        // }
        if (driver1.getRightBumperButton()) {
            slowDown = 0.5;
        } else {
            slowDown = 1;
        }
        Logger.recordOutput("TeleopDrive/slowDown", slowDown);

        Pair<Double, Double> cubicLeft = ControllerUtil.applyExponentialDeadband(
                driver1.getLeftX(),
                driver1.getLeftY(),
                ControllerConfig.driveJoystickDeadband,
                ControllerConfig.joystickExponent);

        double omegaDirection = 0;
        if (driver1.getPOV() == 90) {
            omegaDirection = -1;
        } else if (driver1.getPOV() == 270) {
            omegaDirection = 1;
        }

        double commonXVel_mps = xSlew.calculate(
                SwerveConfig.maxTranslationVel.in(MetersPerSecond) * -cubicLeft.getSecond() * speedShifter * slowDown);
        double commonYVel_mps = ySlew.calculate(
                SwerveConfig.maxTranslationVel.in(MetersPerSecond) * -cubicLeft.getFirst() * speedShifter * slowDown);
        double commmonMaxAngularRate_radps = SwerveConfig.maxAngularVel.in(RadiansPerSecond) * speedShifter;
        double commonOmega_radps = omegaSlew.calculate(commmonMaxAngularRate_radps * omegaDirection);

        Logger.recordOutput("TeleopDrive/commonXVel", commonXVel_mps, MetersPerSecond.name());
        Logger.recordOutput("TeleopDrive/commonYVel", commonYVel_mps, MetersPerSecond.name());
        Logger.recordOutput("TeleopDrive/commonMaxAngularRate", commmonMaxAngularRate_radps, RadiansPerSecond.name());
        Logger.recordOutput("TeleopDrive/commonOmega", commonOmega_radps, RadiansPerSecond.name());

        Rotation2d povDirection_nl = null;
        if (ControllerUtil.isPastDeadband(
                driver1.getRightX(), driver1.getRightY(), ControllerConfig.turnJoystickDeadband)) {
            povDirection_nl = new Rotation2d(
                    ControllerUtil.getFieldSpaceJoystickAngle_rad(driver1.getRightX(), driver1.getRightY()));
        }
        Logger.recordOutput("TeleopDrive/povDirection", povDirection_nl);

        Rotation2d snakeDirection_nl = null;
        if (ControllerUtil.isPastDeadband(
                driver1.getLeftX(), driver1.getLeftY(), ControllerConfig.driveJoystickDeadband)) {
            snakeDirection_nl = new Rotation2d(
                    ControllerUtil.getFieldSpaceJoystickAngle_rad(driver1.getLeftX(), driver1.getLeftY()));
        }
        Logger.recordOutput("TeleopDrive/snakeDirection", snakeDirection_nl);

        boolean usePOV = povDirection_nl != null;
        Logger.recordOutput("TeleopDrive/usePOV", usePOV);
        if (driver1.getXButton()) {
            // brake
            swerve.setControl(brake);
        } else if (ControllerUtil.isPastDeadband(driver1.getLeftTriggerAxis(), ControllerConfig.triggerThreshold)) {
            if (usePOV) {
                // robot centric POV
                swerve.setControl(robotPOV.withVelocityX(commonXVel_mps)
                        .withVelocityY(commonYVel_mps)
                        .withTargetDirection(povDirection_nl)
                        .withMaxAbsRotationalRate(commmonMaxAngularRate_radps));
            } else {
                // robot centric omega
                swerve.setControl(robotOmega
                        .withVelocityX(commonXVel_mps)
                        .withVelocityY(commonYVel_mps)
                        .withRotationalRate(commonOmega_radps));
            }
        } else if (driver1.getLeftBumperButton()) {
            // snake
            if (snakeDirection_nl != null) {
                swerve.setControl(fieldPOV.withVelocityX(commonXVel_mps)
                        .withVelocityY(commonYVel_mps)
                        .withTargetDirection(snakeDirection_nl)
                        .withMaxAbsRotationalRate(commmonMaxAngularRate_radps));
            } else {
                // fallback to field centric omega with a rotational rate of 0
                swerve.setControl(fieldOmega.withVelocityX(commonXVel_mps).withVelocityY(commonYVel_mps));
            }
        } else {
            if (usePOV) {
                // field centric POV
                swerve.setControl(fieldPOV.withVelocityX(commonXVel_mps)
                        .withVelocityY(commonYVel_mps)
                        .withTargetDirection(povDirection_nl)
                        .withMaxAbsRotationalRate(commmonMaxAngularRate_radps));
            } else {
                // field centric omega
                swerve.setControl(fieldOmega
                        .withVelocityX(commonXVel_mps)
                        .withVelocityY(commonYVel_mps)
                        .withRotationalRate(commonOmega_radps));
            }
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
