package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private WebcamName webcam;

    // =========================================================
    // 1. LENS CALIBRATION (REPLACE THESE!)
    // =========================================================
    // Run the "Camera Calibration" OpMode to get these for your 110-degree lens.
    // If you don't, your heading calculation will drift.
    private double fx = 800.0;
    private double fy = 800.0;
    private double cx = 640.0;
    private double cy = 360.0;

    // =========================================================
    // 2. CAMERA LOCATION ON ROBOT
    // =========================================================
    // Where is the camera relative to the center of the wheels?
    // NOTE: X is forward, Y is Left, Z is Up.
    private double camX = 4.0;  // Inches forward of center
    private double camY = 0.0;  // Inches left of center
    private double camZ = 5.0;  // Inches up from ground
    private double camPitch = 0; // Degrees (0 is straight forward)

    // =========================================================
    // 3. DECODE 2026 FIELD TAG POSITIONS (Placeholder IDs 20-24)
    // =========================================================
    // Verify these X/Y/Z coordinates in your Game Manual!
    // Current setup: Standard perimeter tags.
    private static final AprilTagLibrary DECODE_LIBRARY = new AprilTagLibrary.Builder()
            // Blue Alliance Wall Target
            .addTag(20, "Blue Alliance Wall", 4.0, new VectorF(60f, 0f, 6f), DistanceUnit.INCH, new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
            // Red Alliance Wall Target
            .addTag(21, "Red Alliance Wall", 4.0, new VectorF(-60f, 0f, 6f), DistanceUnit.INCH, new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
            // Audience Wall Tags
            .addTag(22, "Audience Wall Left", 4.0, new VectorF(0f, 72f, 4f), DistanceUnit.INCH, new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
            .addTag(23, "Audience Wall Right", 4.0, new VectorF(0f, -72f, 4f), DistanceUnit.INCH, new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
            // Random Field Tag
            .addTag(24, "Center Truss", 4.0, new VectorF(0f, 0f, 12f), DistanceUnit.INCH, new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
            .build();

    public Vision(WebcamName webcamName) {
        webcam = webcamName;
    }

    public void init() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(DECODE_LIBRARY)
                .setLensIntrinsics(fx, fy, cx, cy) // Apply calibration
                .setCameraPose(new Position(DistanceUnit.INCH, camX, camY, camZ, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, camPitch, 0, 0))
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        // 720p 60fps Optimization
        // Decimation = 2 makes it faster (effectively 360p processing) but less range.
        // For Global Shutter at 720p, try 1 (high quality) first, switch to 2 if loop times slow down.
        aprilTag.setDecimation(1);

        // Build the Vision Portal optimized for Arducam Global Shutter
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 720))
                // MJPEG is crucial for high fps on USB cameras
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false) // Disable live view to save CPU
                .setAutoStopLiveView(true)
                .build();

        // Set fast exposure to kill motion blur
        setManualExposure(2, 250);
    }

    /**
     * Manually sets exposure. Global shutter cameras need high gain/low exposure.
     * @param exposureMS Exposure in milliseconds (Try 1-3ms)
     * @param gain Gain (0-255)
     */
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open
        if (visionPortal == null) return;

        // Simple wait loop to ensure camera is ready (runs in init, so it's safe)
        int timeout = 0;
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && timeout < 200) {
            try { Thread.sleep(20); } catch (InterruptedException e) {}
            timeout++;
        }

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            gainControl.setGain(gain);
        }
    }

    /**
     * Calculates the Robot's Heading (in Radians) based on AprilTags.
     * Returns NULL if fewer than 2 tags are visible (low confidence).
     */
    public Double getRelocalizationHeading() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagDetection> validDetections = new ArrayList<>();

        // Filter: Must be a Decode tag (>= 20) and have reasonable confidence margin
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id >= 20 && detection.decisionMargin > 20) {
                validDetections.add(detection);
            }
        }

        // REQUIREMENT: At least 2 tags to be sure
        if (validDetections.size() < 2) {
            return null;
        }

        double totalHeading = 0;
        int count = 0;

        for (AprilTagDetection detection : validDetections) {
            // detection.robotPose is the calculated pose of the ROBOT, not the camera.
            // This works because we set the CameraPose in the builder in init().
            if (detection.robotPose != null) {
                totalHeading += detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
                count++;
            }
        }

        if (count == 0) return null;

        // Return the average heading
        return totalHeading / count;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}