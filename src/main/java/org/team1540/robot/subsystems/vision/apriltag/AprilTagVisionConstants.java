package org.team1540.robot.subsystems.vision.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public class AprilTagVisionConstants {
    public class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(690.876);
        public static final double fieldWidth = Units.inchesToMeters(317);
        public static AprilTagFieldLayout aprilTagFieldLayout;

        static {
            try {
                aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory()
                        .toPath()
                        .resolve("2025-bunnybots.json")
                        .toString());

            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    // Constants +x is front, +y is right(?), +z is up

    public static final String FL_CAMERA_NAME = "front-left";
    public static final String FR_CAMERA_NAME = "front-right";

    public static final Transform3d FL_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(11.491),
            Units.inchesToMeters(11.508),
            Units.inchesToMeters(8.281),
            new Rotation3d(0.0, Math.toRadians(-19), Math.toRadians(-10)));
    public static final Transform3d FR_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(11.491),
            Units.inchesToMeters(-11.508),
            Units.inchesToMeters(8.281),
            new Rotation3d(0.0, Math.toRadians(-19), Math.toRadians(10)));

    public static final double XY_STD_DEV_COEFF = 0.15;
    public static final double ROT_STD_DEV_COEFF = 0.25;

    public static final double MIN_ACCEPTED_NUM_TAGS = 1;
    public static final double MAX_AMBIGUITY = 0.2;
    public static final double MAX_OUTSIDE_OF_FIELD_TOLERANCE = 0.1;
    public static final double MAX_ROBOT_Z_TOLERANCE = 0.5;

    public static final int SIM_RES_WIDTH = 1280;
    public static final int SIM_RES_HEIGHT = 800;
    public static final Rotation2d SIM_DIAGONAL_FOV = Rotation2d.fromDegrees(70);
    public static final double SIM_FPS = 25.0;
    public static final double SIM_AVG_LATENCY_MS = 12.5;
}
