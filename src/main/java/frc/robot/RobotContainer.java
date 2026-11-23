// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CommandSelector;
import frc.robot.commands.auton.PointToPointAutons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagPipeline;
import frc.robot.subsystems.CameraSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandsManager;
import frc.robot.subsystems.LEDControl;

public class RobotContainer {

        /* Setting up bindings for necessary control of the swerve drive platform */

        private final Telemetry logger = new Telemetry(Constants.maxLinVel.in(MetersPerSecond));

        private final CommandXboxController driverCtrl = new CommandXboxController(0);
        private final CommandXboxController operatorCtrl = new CommandXboxController(1);

        // Physical Subsystems
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final LEDControl ledControl = new LEDControl();

        // Autons
        private final PointToPointAutons pointToPointAutons = new PointToPointAutons(drivetrain);

        private final PointToPointAutons.AutonBuilder p2pCAutons = pointToPointAutons.new AutonBuilder();

        // Pathplanner
        SendableChooser<Command> autoChooser;

        // P2P Auton
        SendableChooser<Command> p2pAutoChooser;

        // Mutable Units
        private MutLinearVelocity xVel = MetersPerSecond.mutable(0);
        private MutLinearVelocity yVel = MetersPerSecond.mutable(0);
        private MutAngularVelocity rVel = RotationsPerSecond.mutable(0);

        // Triggers
        private Trigger testMode = new Trigger(DriverStation::isTest);

        // Cameras

        private final AprilTagPipeline leftCamera = new AprilTagPipeline(
                        drivetrain,
                        Constants.leftCameraSettings,
                        "LeftCamera",
                        "LeftCamera");
        private final AprilTagPipeline rightCamera = new AprilTagPipeline(
                        drivetrain,
                        Constants.rightCameraSettings,
                        "RightCamera",
                        "RightCamera");

        // Simulation only
        @SuppressWarnings("unused")
        private final CameraSim cameraSim = RobotBase.isSimulation()
                        ? new CameraSim(drivetrain, leftCamera, rightCamera)
                        : null;

        // Standard Suppliers
        private Supplier<LinearVelocity> fullXVelCtrl = () -> xVel.mut_replace(Constants.maxLinVel)
                        .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getLeftY(), 0.0));
        private Supplier<LinearVelocity> fullYVelCtrl = () -> yVel.mut_replace(Constants.maxLinVel)
                        .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getLeftX(), 0.0));
        private Supplier<AngularVelocity> fullRVelCtrl = () -> rVel.mut_replace(Constants.maxAngVel)
                        .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getRightX(), 0.0));

        private Supplier<LinearVelocity> slowXVelCtrl = () -> xVel.mut_replace(Constants.slowdownLinVel)
                        .mut_times(-driverCtrl.getHID().getLeftY());
        private Supplier<LinearVelocity> slowYVelCtrl = () -> yVel.mut_replace(Constants.slowdownLinVel)
                        .mut_times(-driverCtrl.getHID().getLeftX());
        private Supplier<AngularVelocity> slowRVelCtrl = () -> rVel.mut_replace(Constants.slowdownAngVel)
                        .mut_times(-driverCtrl.getHID().getRightX());

        // Test Command Lists
        private final CommandSelector drivetrainTestCmds = new CommandSelector()
                        .addEntry("SysID Translation Quasistatic Forward",
                                        drivetrain.sysIdTranslationQuasistatic(Direction.kForward))
                        .addEntry("SysID Translation Quasistatic Reverse",
                                        drivetrain.sysIdTranslationQuasistatic(Direction.kReverse))
                        .addEntry("SysID Translation Dynamic Forward",
                                        drivetrain.sysIdTranslationDynamic(Direction.kForward))
                        .addEntry("SysID Translation Dynamic Forward",
                                        drivetrain.sysIdTranslationDynamic(Direction.kReverse))
                        .addEntry("SysID Steer Quasistatic Forward",
                                        drivetrain.sysIdSteerQuasistatic(Direction.kForward))
                        .addEntry("SysID Steer Quasistatic Reverse",
                                        drivetrain.sysIdSteerQuasistatic(Direction.kReverse))
                        .addEntry("SysID Steer Dynamic Forward", drivetrain.sysIdSteerDynamic(Direction.kForward))
                        .addEntry("SysID Steer Dynamic Forward", drivetrain.sysIdSteerDynamic(Direction.kReverse))
                        .addEntry("SysID Rotation Quasistatic Forward",
                                        drivetrain.sysIdRotationQuasistatic(Direction.kForward))
                        .addEntry("SysID Rotation Quasistatic Reverse",
                                        drivetrain.sysIdRotationQuasistatic(Direction.kReverse))
                        .addEntry("SysID Rotation Dynamic Forward", drivetrain.sysIdRotationDynamic(Direction.kForward))
                        .addEntry("SysID Rotation Dynamic Forward", drivetrain.sysIdRotationDynamic(Direction.kReverse))
                        .sendToDashboard("Drivetrain Test Commands");

        /**
         * Constructor
         */
        public RobotContainer() {
                // Init PathPlanner
                initPathPlanner();

                // Configure Robot Bindings
                configureBindings();

                // Initialize drivetrain telemetry
                if (Robot.isSimulation()) {
                        drivetrain.registerTelemetry(
                                        (telemetryFunction) -> logger.telemeterize(telemetryFunction, true));
                } else {
                        drivetrain.registerTelemetry(
                                        (telemetryFunction) -> logger.telemeterize(telemetryFunction, false));
                }
                DriverStation.silenceJoystickConnectionWarning(true);
        }

        /**
         * Initializes PathPlanner
         */
        private void initPathPlanner() {
                // Initialize Auton chooser

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auton Chooser", autoChooser);

                p2pAutoChooser = pointToPointAutons.getAutonChooser();
                SmartDashboard.putData("P2P Auton Chooser", p2pAutoChooser);

        }

        /**
         * Configures robot bindings
         */
        private void configureBindings() {
                drivetrainBindings();
                miscBindings();
        }

        /**
         * Configures drivetrain bindings
         */
        private void drivetrainBindings() {
                // Set default drivetrain command
                drivetrain.setDefaultCommand(
                                drivetrain.getDriveCmd(
                                                () -> driverCtrl.getHID().getRightBumperButton() ? fullXVelCtrl.get()
                                                                : slowXVelCtrl.get(),
                                                () -> driverCtrl.getHID().getRightBumperButton() ? fullYVelCtrl.get()
                                                                : slowYVelCtrl.get(),
                                                () -> driverCtrl.getHID().getRightBumperButton() ? fullRVelCtrl.get()
                                                                : slowRVelCtrl.get()));

                driverCtrl.leftTrigger(.1).whileTrue(
                                drivetrain.autoAlignCmd(() -> new Pose2d(
                                                FieldLayout.Hub.redHubCenter
                                                                .plus(new Translation2d(Inches.of(75), Meters.zero())),
                                                Rotation2d.fromDegrees(180))));

                driverCtrl.povUp().onTrue(drivetrain.runOnce(
                                () -> drivetrain.resetPose(
                                                new Pose2d(
                                                                FieldLayout.Hub.getHubCenterFront(),
                                                                Rotation2d.fromDegrees(FieldLayout.getForwardAngle()
                                                                                .in(Degrees) + 180)))));

                driverCtrl.x().onTrue(drivetrain.brakeCmd().until(() -> driverCtrl
                                .axisMagnitudeGreaterThan(0, 0.06)
                                .or(driverCtrl.axisMagnitudeGreaterThan(1, 0.06))
                                .or(driverCtrl.axisGreaterThan(4, 0.06))
                                .or(driverCtrl.axisMagnitudeGreaterThan(5, 0.06))
                                .getAsBoolean()));

                // Idle motors when disabled
                RobotModeTriggers.disabled().whileTrue(drivetrain.idleCmd());

                // Drivetrain test Controls
                testMode.and(driverCtrl.a()).whileTrue(drivetrainTestCmds.runCommandCmd());
        }

        private void miscBindings() {

        }

        /**
         * Retrieves the selected auton
         * 
         * @return the selected auton
         */
        public Command getAutonomousCommand() {
                SmartDashboard.putString("Current Auton", autoChooser.getSelected().getName());
                return autoChooser.getSelected();
        }

        public Command getP2PAutononomousCmd() {
                SmartDashboard.putString("Current P2P Auton", p2pAutoChooser.getSelected().getName());
                return p2pAutoChooser.getSelected();
        }

        public Command getP2PCAutonomousCmd() {
                return p2pCAutons.getAuton();
        }

}
