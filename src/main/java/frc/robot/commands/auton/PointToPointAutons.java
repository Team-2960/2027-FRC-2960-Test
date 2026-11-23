package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Util.WaypointFactory;
import frc.robot.Util.WaypointFactory.AllianceFlip;
import frc.robot.Util.WaypointFactory.Waypoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandsManager;
public final class PointToPointAutons {

    private final CommandSwerveDrivetrain drivetrain;

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();
    
    public PointToPointAutons(CommandSwerveDrivetrain drivetrain){
        
        this.drivetrain = drivetrain;
        
        autonChooser.setDefaultOption("null", Commands.none());
        autonChooser.addOption("Test Auton", getTestAuton());
    }

    public static final class AutonWaypoints{
        //Trench
        public static final Waypoint rightTrenchAutonStart = WaypointFactory.of(new Pose2d(FieldLayout.Trench.blueTrenchRight, Rotation2d.fromDegrees(90)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightNeutralIntakePrep = WaypointFactory.of(new Pose2d(7.8, FieldLayout.Trench.blueTrenchRight.getY(), Rotation2d.fromDegrees(65)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightNeutralIntakeFinished = WaypointFactory.of(new Pose2d(8.4, 3.8, Rotation2d.fromDegrees(85)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightBumpCrossPrep = WaypointFactory.of(new Pose2d(6.0, FieldLayout.Bump.blueBumpRight.getY(), Rotation2d.fromDegrees(180)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightBumpCrossFinished = rightBumpCrossPrep.translated(-3.5, 0).withRotation(Rotation2d.fromDegrees(210));
        public static final Waypoint rightTrenchNeutralPrep = rightTrenchAutonStart.translated(-2, -0.2).withRotation(Rotation2d.fromDegrees(90));
        public static final Waypoint rightTrenchAlliancePrep = rightTrenchAutonStart.translated(2, 0).withRotation(Rotation2d.fromRadians(-90));

        public static final Waypoint rightBumpNeutralPrep = rightBumpCrossFinished.translated(1, 0).withRotation(Rotation2d.kZero);

        //Snake Intake Path Points
        public static final Waypoint snakePoint1 = WaypointFactory.of(new Pose2d(6.4, 3.6, Rotation2d.fromDegrees(-160)), AllianceFlip.ALLIANCE);
        public static final Waypoint snakePoint2 = rightBumpCrossPrep;

        public static final Waypoint cornerPoint = WaypointFactory.of(new Pose2d(0.5, 0.5, Rotation2d.kZero), AllianceFlip.ALLIANCE);
        public static final Waypoint hubEndPoint = WaypointFactory.of(new Pose2d(3.22, 4, Rotation2d.kZero), AllianceFlip.ALLIANCE);
        

        public static final Waypoint rightBumpAutonStart = WaypointFactory.of(
            FieldLayout.Bump.blueBumpRight.minus(
                new Translation2d(FieldLayout.Bump.bumpWidth.div(2).plus(Constants.robotWithBumpersLength.div(2)), 
                    Inches.zero())), 
            Rotation2d.kZero, 
            AllianceFlip.ALLIANCE);

        public static final Waypoint centerHubAutonStart = WaypointFactory.of(
            FieldLayout.Hub.blueHubCenterFront.minus(
                new Translation2d(
                    Constants.robotWithBumpersLength.div(2), Inches.of(0))), 
            Rotation2d.kZero, 
            AllianceFlip.ALLIANCE);

        public static final Pose2d[] rightAutonPoints = {
            rightTrenchAutonStart.get(),
            rightNeutralIntakePrep.get(),
            rightNeutralIntakeFinished.get(),
            rightBumpCrossPrep.get(),
            rightBumpCrossFinished.get(),
            rightTrenchNeutralPrep.get()
        };
    }

    public static class WaypointSet {
        private final List<Waypoint> waypoints = new ArrayList<>();

        public Waypoint add(Waypoint w) {
            waypoints.add(w);
            return w; // returns the waypoint so it can be used inline
        }

        public List<Pose2d> getResolved() {
            return waypoints.stream().map(Waypoint::get).toList();
        }

        public Pose2d[] getResolvedArray() {
            return waypoints.stream().map(Waypoint::get).toArray(Pose2d[]::new);
        }
    }

    public final SendableChooser<Command> getAutonChooser(){
        return autonChooser;
    }

    public final Command eventMarkerGoTo(Waypoint target, double percent, Distance tolerance, Command event){
        return drivetrain.runOnce(() -> drivetrain.initialPoseHelper(drivetrain.getPose2d()))
        .andThen(
            new ParallelDeadlineGroup(
                drivetrain.goToPointCmd(target, tolerance),
                new SequentialCommandGroup(
                    Commands.waitUntil(() -> {
                        Pose2d targetPose = target.get();
                        double totalDist = PhotonUtils.getDistanceToPose(drivetrain.getInitialPose(), targetPose);
                        double remainingDist = PhotonUtils.getDistanceToPose(drivetrain.getPose2d(), targetPose);
                        double coveredDist = totalDist - remainingDist;
                        return coveredDist >= totalDist * percent;
                    }),
                    event
                )
            )
        );
    }
 
    public Command getTestAuton(){
        return new SequentialCommandGroup(
            drivetrain.getResetPoseCmd(AutonWaypoints.rightTrenchAutonStart.withRotation(FieldLayout.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)),
            drivetrain.goToPointCmd(AutonWaypoints.rightNeutralIntakePrep, Meters.of(0.2)),
            drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(3), AutonWaypoints.rightNeutralIntakeFinished, Meters.of(0.2)),
            drivetrain.goToPointCmd(AutonWaypoints.rightBumpCrossPrep, Meters.of(1)),
            drivetrain.goToPointCmd(AutonWaypoints.rightBumpCrossFinished, Meters.of(0.2))
        );
    }

    // Tiny alias to keep the impl method readable
    private Waypoint w(Waypoint waypoint, boolean mirror) {
        return mirror ? mirrored(waypoint) : waypoint;
    }

    /**
     * Applies an additional flip policy on top of a waypoint's existing policy.
     * Used to mirror entire autons without redefining every waypoint.
     */
    private Waypoint mirrored(Waypoint w) {
        return switch (w.getFlipPolicy()) {
            case NONE     -> w.withFlipPolicy(AllianceFlip.X_AXIS);
            case ALLIANCE -> w.withFlipPolicy(AllianceFlip.BOTH);
            case X_AXIS   -> w.withFlipPolicy(AllianceFlip.NONE);
            case BOTH     -> w.withFlipPolicy(AllianceFlip.ALLIANCE);
        };
    }

    public class AutonBuilder{

        @SuppressWarnings("rawtypes")
        private final ArrayList<SendableChooser<Boolean>> isMirroredChooserList = new ArrayList<>();

        private final SendableChooser<StartType> startTypeChooser = new SendableChooser<>();

        @SuppressWarnings("rawtypes")
        private final ArrayList<SendableChooser<NeutralZoneType>> neutralZoneChooserList = new ArrayList<>();

        @SuppressWarnings("rawtypes")
        private final ArrayList<SendableChooser<IntakePathType>> intakePathChooserList = new ArrayList<>();

        @SuppressWarnings("rawtypes")
        private final ArrayList<SendableChooser<ReturnType>> returnTypeChooserList = new ArrayList<>();

        @SuppressWarnings("rawtypes")
        private final ArrayList<SendableChooser<Boolean>> repeatCycleChooserList = new ArrayList<>();

        private final ArrayList<SendableChooser<EndType>> endTypeChooserList = new ArrayList<>();

        private final ArrayList<GenericEntry> startDelayList = new ArrayList<>();
        
        private final ArrayList<GenericEntry> returnDelayList = new ArrayList<>();

        private final ArrayList<GenericEntry> shootTimeList = new ArrayList<>();

        private final ArrayList<GenericEntry> endDelayList = new ArrayList<>();

        private final int cycleCount = 3;


        public AutonBuilder(){
            for (int i = 0; i < cycleCount; i++){

                SendableChooser<Boolean> isMirroredChooser = new SendableChooser<>();
                SendableChooser<NeutralZoneType> neutralZoneChooser = new SendableChooser<>();
                SendableChooser<IntakePathType> intakePathChooser = new SendableChooser<>();
                SendableChooser<ReturnType> returnTypeChooser = new SendableChooser<>();
                SendableChooser<Boolean> repeatCycleChooser = new SendableChooser<>();
                SendableChooser<EndType> endTypeChooser = new SendableChooser<>();

                GenericEntry startDelay;
                GenericEntry returnDelay;
                GenericEntry shootTime;
                GenericEntry endDelayTime;

                startTypeChooser.addOption("Trench", StartType.TRENCH);
                startTypeChooser.addOption("Bump", StartType.BUMP);
                startTypeChooser.addOption("Hub", StartType.HUB);
                startTypeChooser.setDefaultOption("None", StartType.NONE);

                neutralZoneChooser.addOption("Trench", NeutralZoneType.TRENCH);
                neutralZoneChooser.addOption("Bump", NeutralZoneType.BUMP);
                neutralZoneChooser.setDefaultOption("None", NeutralZoneType.NONE);

                intakePathChooser.addOption("Snake", IntakePathType.SNAKE);
                intakePathChooser.addOption("Straight", IntakePathType.STRAIGHT);
                intakePathChooser.setDefaultOption("None", IntakePathType.NONE);

                returnTypeChooser.addOption("Trench", ReturnType.TRENCH);
                returnTypeChooser.addOption("Bump", ReturnType.BUMP);
                returnTypeChooser.setDefaultOption("None", ReturnType.NONE);

                isMirroredChooser.addOption("Right", false);
                isMirroredChooser.setDefaultOption("Left", true);

                repeatCycleChooser.addOption("True", true);
                repeatCycleChooser.setDefaultOption("False", false);

                endTypeChooser.setDefaultOption("None", EndType.NONE);
                endTypeChooser.addOption("Corner", EndType.CORNER);
                endTypeChooser.addOption("Hub", EndType.HUB);

                // SmartDashboard.putData("Start Type P2PC", startTypeChooser);
                // SmartDashboard.putData("Neutral Zone P2PC", neutralZoneChooser);
                // SmartDashboard.putData("Intake Path P2PC", intakePathChooser);
                // SmartDashboard.putData("Return Type P2PC", returnTypeChooser);
                // SmartDashboard.putData("Is Mirrored P2PC", isMirroredChooser);

                // SmartDashboard.putNumber("Start Delay P2PC", 0);
                // SmartDashboard.putNumber("Return Delay P2PC", 0);


                var firstCycleLayout = Shuffleboard.getTab("Auton")
                    .getLayout("Cycle " + (i + 1) + " P2PC", BuiltInLayouts.kList)
                    .withSize(2, 5);

                if (i == 0){
                    firstCycleLayout.add("Start Type P2PC", startTypeChooser);
                }
                
                firstCycleLayout.add("Neutral Zone P2PC", neutralZoneChooser);
                firstCycleLayout.add("Intake Path P2PC", intakePathChooser);
                firstCycleLayout.add("Return Type P2PC", returnTypeChooser);
                firstCycleLayout.add("Left or Right P2PC", isMirroredChooser);
                firstCycleLayout.add("End Type P2PC", endTypeChooser);

                startDelay = firstCycleLayout.add("Start Delay P2PC", 0).getEntry();
                returnDelay = firstCycleLayout.add("Return Delay P2PC", 0).getEntry();
                shootTime = firstCycleLayout.add("Shoot Time", 4).getEntry();
                endDelayTime = firstCycleLayout.add("End Delay P2PC", 0).getEntry();

                isMirroredChooserList.add(isMirroredChooser);
                neutralZoneChooserList.add(neutralZoneChooser);
                intakePathChooserList.add(intakePathChooser);
                returnTypeChooserList.add(returnTypeChooser);
                repeatCycleChooserList.add(repeatCycleChooser);
                endTypeChooserList.add(endTypeChooser);
                
                startDelayList.add(startDelay);
                returnDelayList.add(returnDelay);
                shootTimeList.add(shootTime);
                endDelayList.add(endDelayTime);
            }
        }

        private enum StartType{
            NONE,
            TRENCH,
            BUMP,
            HUB
        }

        private enum NeutralZoneType{
            NONE,
            TRENCH,
            BUMP
        }

        private enum IntakePathType{
            NONE,
            SNAKE,
            STRAIGHT        
        }

        private enum ReturnType{
            NONE,
            TRENCH,
            BUMP
        }

        private enum EndType{
            NONE,
            CORNER,
            HUB
        }

        public Command getStartCommand(){
            Waypoint trench = w(AutonWaypoints.rightTrenchAutonStart, isMirroredChooserList.get(0).getSelected());
            Waypoint bump = w(AutonWaypoints.rightBumpAutonStart, isMirroredChooserList.get(0).getSelected());
            Waypoint hub = w(AutonWaypoints.centerHubAutonStart, isMirroredChooserList.get(0).getSelected());
            
            switch (startTypeChooser.getSelected()) {
                case TRENCH:
                    return drivetrain.getResetPoseAllianceCmd(trench);
                
                case BUMP:
                    return drivetrain.getResetPoseAllianceCmd(bump);

                case HUB:
                    return drivetrain.getResetPoseAllianceCmd(hub);
                
                default:
                    return drivetrain.getResetPoseAllianceCmd(Pose2d.kZero);
            }
        }

        public Command getAuton(){
            // Command firstCycle =  new SequentialCommandGroup(
            //     getStartCommand(),
            //     getStartDelayCmd(),
            //     getNeutralZoneCommand(),
            //     getIntakePathCommand(),
            //     getReturnDelayCmd(),
            //     getReturnCommand(),
            //     getShootRoutine()
            // );

            Command finalAuton = getStartCommand();

            for (int i = 0; i < cycleCount; i++){
                finalAuton = finalAuton.andThen(
                );
            }

            return finalAuton;
        }
        
        // public SendableChooser<StartType> getStartTypeChooser(){
        //     return startTypeChooser;
        // }

        // public SendableChooser<NeutralZoneType> getNeutralZoneChooser(){
        //     return neutralZoneChooser;
        // }
        
        // public SendableChooser<IntakePathType> getIntakePathChooser(){
        //     return intakePathChooser;
        // }

        // public SendableChooser<ReturnType> getReturnTypeChooser(){
        //     return returnTypeChooser;
        // }
    }
}