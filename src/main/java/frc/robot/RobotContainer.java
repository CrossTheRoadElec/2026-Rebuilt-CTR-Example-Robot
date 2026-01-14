// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Flywheel.FlywheelSetpoint;
import frc.robot.subsystems.Intake.IntakeSetpoint;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Flywheel flywheel = new Flywheel();
    public final Intake intake = new Intake();

    private final AngularVelocity SpinUpThreshold = RotationsPerSecond.of(3); // Tune to increase accuracy while not sacrificing throughput
    /* The flywheel is ready to shoot when it's near the target or when the driver overrides it with the X button */
    private final Trigger isFlywheelReadyToShoot = flywheel.getTriggerWhenNearTarget(SpinUpThreshold).or(joystick.x());

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Stop Shooting", flywheel.coastFlywheel().alongWith(intake.coastIntake()));
        /* Shoot commands need a bit of time to spool up the flywheel before feeding with the intake */
        NamedCommands.registerCommand("Shoot Near", flywheel.setTarget(() -> FlywheelSetpoint.Near)
                                                            .alongWith(Commands.waitUntil(isFlywheelReadyToShoot).andThen(intake.setTarget(() ->IntakeSetpoint.FeedToShoot))));
        NamedCommands.registerCommand("Shoot Far", flywheel.setTarget(() -> FlywheelSetpoint.Far)
                                                            .alongWith(Commands.waitUntil(isFlywheelReadyToShoot).andThen(intake.setTarget(() ->IntakeSetpoint.FeedToShoot))));
        NamedCommands.registerCommand("Stop Intake", intake.coastIntake().alongWith(flywheel.coastFlywheel()));
        NamedCommands.registerCommand("Intake Fuel", intake.setTarget(() -> IntakeSetpoint.Intake).alongWith(flywheel.setTarget(()-> FlywheelSetpoint.Intake)));
        NamedCommands.registerCommand("Outtake Fuel", intake.setTarget(() -> IntakeSetpoint.Outtake).alongWith(flywheel.setTarget(()-> FlywheelSetpoint.Outtake)));

        autoChooser = AutoBuilder.buildAutoChooser("Only Score");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Bind the start button to set the field-centric forward in case it's lost for whatever reason.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Bind left bumper/trigger to our intake/outtake
        joystick.leftBumper().whileTrue(intake.setTarget(()->IntakeSetpoint.Intake).alongWith(flywheel.setTarget(()->FlywheelSetpoint.Intake)));
        joystick.leftTrigger().whileTrue(intake.setTarget(()->IntakeSetpoint.Outtake).alongWith(flywheel.setTarget(()->FlywheelSetpoint.Outtake)));

        // Bind right bumper/trigger to our near/far shots
        joystick.rightBumper().whileTrue(
            flywheel.setTarget(()->FlywheelSetpoint.Near) // First spin up the flywheel
            .alongWith(Commands.waitUntil(isFlywheelReadyToShoot).andThen(intake.setTarget(()->IntakeSetpoint.FeedToShoot)))
        );
        joystick.rightTrigger().whileTrue(
            flywheel.setTarget(()->FlywheelSetpoint.Far) // First spin up the flywheel
            .alongWith(Commands.waitUntil(isFlywheelReadyToShoot).andThen(intake.setTarget(()->IntakeSetpoint.FeedToShoot)))
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void consumePhotonVisionMeasurement(EstimatedRobotPose pose) {
        /* Super simple, should modify to support variable standard deviations */
        drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }
}
