package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

public class PhotonVisionSystem {
    final AprilTagFieldLayout TagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    /* IDs 3,4 and 19,20 are on the side of the hub that we can't shoot from, so don't include them */
    private final int[] RedHubApriltagIds = new int[]{
        2, /* 3, 4, */ 5, 8, 9, 10, 11
    };
    private final int[] BlueHubApriltagIds = new int[]{
        18, /* 19, 20, */ 21, 24, 25, 26, 27
    };

    private final ApriltagTarget RedHub = new ApriltagTarget(Map.of(
        2, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        5, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        8, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
        9, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
        10, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        11, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))
    ));

    private final ApriltagTarget BlueHub = new ApriltagTarget(Map.of(
        18, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        21, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        24, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
        25, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
        26, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        27, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))
    ));

    PhotonCamera targetCamera = new PhotonCamera("camera");
    Transform3d robotToCamera = new Transform3d(
        /* X, Y, Z */
        new Translation3d(Meters.of(-0.5), Meters.of(0), Meters.of(0.5)),
        /* Roll, Pitch, Yaw */
        /* A pitch of -40 degrees appears to see the apriltags  */
        new Rotation3d(Degrees.of(0), Degrees.of(-40), Degrees.of(180)));
    
    /* Use the current robot heading to keep track of where to target when aiming for the hub */
    Supplier<Pose2d> currentRobotPose;

    PhotonPoseEstimator estimator = new PhotonPoseEstimator(TagLayout, robotToCamera);
    Consumer<LoggableRobotPose> poseConsumer;

    PhotonCameraSim cameraSim = new PhotonCameraSim(targetCamera);
    VisionSystemSim visionSim = new VisionSystemSim("Camera Sim");
    LoggableRobotPose[] allPoses = new LoggableRobotPose[0];
    double timeOfLastTrackedHubTarget = 0;
    PhotonTrackedTarget lastTrackedHubTarget = new PhotonTrackedTarget(0, 0, 0, 0, -1, -1, 0, Transform3d.kZero, Transform3d.kZero, 0, new ArrayList<TargetCorner>(), new ArrayList<TargetCorner>());
    Pose3d hubTarget = Pose3d.kZero;
    Rotation2d hubHeading = Rotation2d.kZero;

    /* Hoot replay/autologging */
    private final HootAutoReplay autoReplay = new HootAutoReplay()
        /* We need to fetch the latest result from the photoncamera when not replaying, otherwise we need to fill the list of results when we are replaying */
        .withStructArray("Camera/PoseEstimations", LoggableRobotPose.struct, () -> allPoses, val -> allPoses = val.value)
        .withStruct("Camera/HubTarget", Pose3d.struct, () -> hubTarget, val -> hubTarget = val.value)
        .withStruct("Camera/HubHeading", Rotation2d.struct, () -> hubHeading, val -> hubHeading = val.value)
        .withProtobuf("Camera/LastTrackedHubTarget", PhotonTrackedTarget.proto, () -> lastTrackedHubTarget, val -> lastTrackedHubTarget = val.value)
        .withDouble("Camera/LastTrackedHubTargetTime", () -> timeOfLastTrackedHubTarget, val -> timeOfLastTrackedHubTarget = val.value);

    private final NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("CameraDetails");
    private final StructPublisher<Pose3d> hubTargetPublisher = cameraTable.getStructTopic("HubTarget", Pose3d.struct).publish();
    private final StructPublisher<Rotation2d> hubHeadingPublisher = cameraTable.getStructTopic("HubHeading", Rotation2d.struct).publish();
    
    public PhotonVisionSystem(Consumer<LoggableRobotPose> poseConsumer, Supplier<Pose2d> currentRobotPose) {
        this.poseConsumer = poseConsumer;
        this.currentRobotPose = currentRobotPose;


        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(TagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(targetCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToCamera);

            cameraSim.enableDrawWireframe(true);
        }
    }

    public void periodic() {
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (!Utils.isReplay()) {
            /* If this is not replay, get the hardware/simulated results from the camera */
            var allResults = targetCamera.getAllUnreadResults();
            var estimates = new ArrayList<LoggableRobotPose>();

            int[] hubTargetIds;
            /* Figure out if we should use red alliance hub ids or blue alliance hub ids */
            if (currentAlliance == Alliance.Red) {
                hubTargetIds = RedHubApriltagIds;
            } else {
                hubTargetIds = BlueHubApriltagIds;
            }

            /* Process them */
            for (var result : allResults) {
                var allTargets = result.getTargets();
                /* Pick the target with the lowest ambiguity */
                PhotonTrackedTarget bestTarget = null;
                for (PhotonTrackedTarget target : allTargets) {
                    /* Check that the apriltag id is a hub ID */
                    if (Arrays.stream(hubTargetIds).anyMatch(x -> x == target.fiducialId)) {
                        /* If we've never assigned the best target, use this one */
                        if (bestTarget == null) {
                            bestTarget = target;
                        }
                        /* Otherwise only update the target if this is a better ambiguity */
                        else if (target.poseAmbiguity < bestTarget.poseAmbiguity && target.poseAmbiguity > 0) {
                            bestTarget = target;
                        }
                    }
                }
                /* And if we have a best target, use it */
                if (bestTarget != null) {
                    lastTrackedHubTarget = bestTarget;
                    timeOfLastTrackedHubTarget = Utils.getCurrentTimeSeconds();
                    Transform3d tagRelativeToRobot = lastTrackedHubTarget.bestCameraToTarget;
                    var transformToHub = currentAlliance == Alliance.Red ? RedHub.getHubPose(lastTrackedHubTarget.fiducialId) :
                                                          BlueHub.getHubPose(lastTrackedHubTarget.fiducialId);
                    var robotPose = currentRobotPose.get();
                    hubTarget = new Pose3d(robotPose).transformBy(robotToCamera).transformBy(tagRelativeToRobot).transformBy(transformToHub);
                    var hubRelativeToRobot = hubTarget.relativeTo(new Pose3d(robotPose));
                    hubHeading = robotPose.getRotation().plus(hubRelativeToRobot.getTranslation().toTranslation2d().getAngle()
                            .plus(currentAlliance == Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero));
                }

                var estimate = estimator.estimateCoprocMultiTagPose(result);
                if (estimate.isEmpty()) {
                    estimate = estimator.estimateLowestAmbiguityPose(result);
                }
                estimate.ifPresent(val -> estimates.add(new LoggableRobotPose(val.estimatedPose, val.timestampSeconds)));
            }

            /* And save them so we can feed them to the drivetrain */
            allPoses = estimates.toArray(new LoggableRobotPose[0]);
        }

        /* Auto-log the poses as they come in, or pull them from the log if we're in replay */
        autoReplay.update();

        /* And process every pose we got */
        for(LoggableRobotPose pose : allPoses) {
            poseConsumer.accept(pose);
        }
        hubTargetPublisher.accept(hubTarget);
        hubHeadingPublisher.accept(hubHeading);
    }

    public void simPeriodic(Pose2d simPose) {
        visionSim.update(simPose);
    }
    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
    public boolean isHubTargetValid() {
        return Utils.getCurrentTimeSeconds() - timeOfLastTrackedHubTarget < 0.2;
    }
    public Pose3d getHubPoseRelativeToRobot() {
        return hubTarget;
    }
    public Rotation2d getHeadingToHubFieldRelative() {
        return hubHeading;
    }
}
