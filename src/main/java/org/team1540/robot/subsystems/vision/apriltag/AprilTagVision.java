package org.team1540.robot.subsystems.vision.apriltag;

import static org.team1540.robot.subsystems.vision.apriltag.AprilTagVisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.RobotState;
import org.team1540.robot.subsystems.vision.apriltag.AprilTagVisionIO.PoseObservation;

public class AprilTagVision extends SubsystemBase {

    private final AprilTagVisionIO[] visionIOs;
    private final AprilTagVisionIOInputsAutoLogged[] cameraInputs;

    private final Alert[] disconnectedAlerts;

    private final ArrayList<Pose3d> lastAcceptedPoses = new ArrayList<>();
    private final ArrayList<Pose3d> lastRejectedPoses = new ArrayList<>();
    private final ArrayList<Pose3d> lastSeenTagPoses = new ArrayList<>();

    private AprilTagVision(AprilTagVisionIO... visionIOs) {
        this.visionIOs = visionIOs;

        this.cameraInputs = new AprilTagVisionIOInputsAutoLogged[visionIOs.length];
        this.disconnectedAlerts = new Alert[visionIOs.length];

        for (int count = 0; count < cameraInputs.length; count++) {
            cameraInputs[count] = new AprilTagVisionIOInputsAutoLogged();
            disconnectedAlerts[count] =
                    new Alert(visionIOs[count].name + " is disconnected.", Alert.AlertType.kWarning);
        }
    }

    public void periodic() {
        for (int count = 0; count < visionIOs.length; count++) {
            visionIOs[count].updateInputs(cameraInputs[count]);
            Logger.processInputs("Vision/" + visionIOs[count].name, cameraInputs[count]);
        }

        RobotState robotState = RobotState.getInstance();

        lastAcceptedPoses.clear();
        lastRejectedPoses.clear();
        lastSeenTagPoses.clear();

        for (int count = 0; count < visionIOs.length; count++) {
            disconnectedAlerts[count].set(!cameraInputs[count].connected);

            // global
            for (PoseObservation observation : cameraInputs[count].poseObservations) {
                if (robotState.addVisionMeasurement(observation)) {
                    lastAcceptedPoses.add(observation.estimatedPoseMeters());
                } else {
                    lastRejectedPoses.add(observation.estimatedPoseMeters());
                }

                lastSeenTagPoses.addAll(Arrays.stream(cameraInputs[count].seenTagIDs)
                        .mapToObj(tagID -> AprilTagVisionConstants.FieldConstants.aprilTagFieldLayout
                                .getTagPose(tagID)
                                .orElse(Pose3d.kZero))
                        .toList());
            }

            Logger.recordOutput("Vision/AcceptedPoses", lastAcceptedPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/RejectedPoses", lastRejectedPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/SeenTagPoses", lastSeenTagPoses.toArray(new Pose3d[0]));
        }
    }

    public static AprilTagVision createReal() {
        return new AprilTagVision(
                new AprilTagVisionIOPhoton(FL_CAMERA_NAME, FL_CAMERA_TRANSFORM),
                new AprilTagVisionIOPhoton(FR_CAMERA_NAME, FR_CAMERA_TRANSFORM));
    }

    public static AprilTagVision createDummyIO() {
        return new AprilTagVision(new AprilTagVisionIO(FL_CAMERA_NAME), new AprilTagVisionIO(FR_CAMERA_NAME));
    }
}
