package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ControllerUtil;
import org.littletonrobotics.junction.Logger;

public class TeleopDrive extends Command {

    private final Swerve swerve;

    private final SlewRateLimiter xSlew =
            new SlewRateLimiter(SwerveConstants.teleopTranslationAcceleration.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter ySlew =
            new SlewRateLimiter(SwerveConstants.teleopTranslationAcceleration.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter omegaSlew =
            new SlewRateLimiter(SwerveConstants.teleopAngularAcceleration.in(RadiansPerSecondPerSecond));

    private final PIDController headingController =
            new PIDController(SwerveConstants.headingP, SwerveConstants.headingI, SwerveConstants.headingD);

    public TeleopDrive(Swerve _swerve) {
        swerve = _swerve;
        addRequirements(swerve);

        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        headingController.reset();
    }

    @Override
    public void execute() {
        XboxController driver1 = RobotContainer.instance().driver1;
        // XboxController driver2 = RobotContainer.instance().driver2;

        double speedShifter =
                ControllerUtil.isPastDeadband(driver1.getRightTriggerAxis(), ControllerConstants.triggerThreshold)
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
                ControllerConstants.driveJoystickDeadband,
                ControllerConstants.joystickExponent);

        double omegaDirection = 0;
        if (driver1.getPOV() == 90) {
            omegaDirection = -1;
        } else if (driver1.getPOV() == 270) {
            omegaDirection = 1;
        }

        double commonXVel_mps = xSlew.calculate(SwerveConstants.maxTranslationVel.in(MetersPerSecond)
                * -cubicLeft.getSecond()
                * speedShifter
                * slowDown);
        double commonYVel_mps = ySlew.calculate(SwerveConstants.maxTranslationVel.in(MetersPerSecond)
                * -cubicLeft.getFirst()
                * speedShifter
                * slowDown);
        double commmonMaxAngularRate_radps = SwerveConstants.maxAngularVel.in(RadiansPerSecond) * speedShifter;
        double ommegaControl_radps = omegaSlew.calculate(commmonMaxAngularRate_radps * omegaDirection);

        Logger.recordOutput("TeleopDrive/commonXVel", commonXVel_mps, MetersPerSecond.name());
        Logger.recordOutput("TeleopDrive/commonYVel", commonYVel_mps, MetersPerSecond.name());
        Logger.recordOutput("TeleopDrive/commonMaxAngularRate", commmonMaxAngularRate_radps, RadiansPerSecond.name());

        if (driver1.getXButton()) {
            swerve.stopWithX();
            Logger.recordOutput("TeleopDrive/headingDirection", Double.NaN, Radians.name());
            Logger.recordOutput("TeleopDrive/targetOmega", Double.NaN, RadiansPerSecond.name());
            headingController.reset();
        } else {
            double targetOmega_radps;
            if (driver1.getLeftBumperButton()) {
                // snake orientation
                if (ControllerUtil.isPastDeadband(
                        driver1.getLeftX(), driver1.getLeftY(), ControllerConstants.driveJoystickDeadband)) {
                    double heading_rad =
                            ControllerUtil.getFieldSpaceJoystickAngle_rad(driver1.getLeftX(), driver1.getLeftY());
                    Logger.recordOutput("TeleopDrive/headingDirection", heading_rad, Radians.name());

                    targetOmega_radps =
                            headingController.calculate(swerve.getRotation().getRadians(), heading_rad);
                } else {
                    // fallback to no rotation
                    targetOmega_radps = 0;
                    Logger.recordOutput("TeleopDrive/headingDirection", Double.NaN, Radians.name());
                }
            } else if (omegaDirection != 0) {
                // omega control with dpad
                targetOmega_radps = ommegaControl_radps;
                Logger.recordOutput("TeleopDrive/headingDirection", Double.NaN, Radians.name());

                headingController.reset();
            } else if (ControllerUtil.isPastDeadband(
                    driver1.getRightX(), driver1.getRightY(), ControllerConstants.turnJoystickDeadband)) {
                // joystick orientation
                double heading_rad =
                        ControllerUtil.getFieldSpaceJoystickAngle_rad(driver1.getRightX(), driver1.getRightY());
                Logger.recordOutput("TeleopDrive/headingDirection", heading_rad, Radians.name());

                targetOmega_radps =
                        headingController.calculate(swerve.getRotation().getRadians(), heading_rad);

            } else {
                // fallback to no rotation
                targetOmega_radps = 0;
                Logger.recordOutput("TeleopDrive/headingDirection", Double.NaN, Radians.name());
            }

            targetOmega_radps = Math.copySign(
                    Math.min(Math.abs(targetOmega_radps), commmonMaxAngularRate_radps), targetOmega_radps);
            Logger.recordOutput("TeleopDrive/targetOmega", targetOmega_radps, RadiansPerSecond.name());

            if (ControllerUtil.isPastDeadband(driver1.getLeftTriggerAxis(), ControllerConstants.triggerThreshold)) {
                // robot centric
                swerve.runRobotRelativeVelocity(new ChassisSpeeds(commonXVel_mps, commonYVel_mps, targetOmega_radps));
            } else {
                // operator centric
                swerve.runOperatorRelativeVelocity(
                        new ChassisSpeeds(commonXVel_mps, commonYVel_mps, targetOmega_radps));
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
