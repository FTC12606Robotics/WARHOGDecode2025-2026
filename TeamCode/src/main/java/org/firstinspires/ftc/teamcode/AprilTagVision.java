package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
  Wrapper class for FTC VisionPortal AprilTag detection.
  Lets you easily start, stop, and fetch detections from your webcam.
 */
public class AprilTagVision {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    //Constructor: builds the AprilTag processor and VisionPortal.
    public AprilTagVision(HardwareMap hardwareMap) {
        //Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // FTC tag family
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();

        //Create VisionPortal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))   //set resolution here
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)       //YUY2 uses less CPU than MJPEG
                .addProcessor(aprilTag)
                .build();
    }

    //Returns the list of currently detected tags.
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    //Convenience: returns the first detected tag, or null if none.
    public AprilTagDetection getBestTag() {
        List<AprilTagDetection> detections = getDetections();
        return detections.isEmpty() ? null : detections.get(0);
    }

    //Stops the VisionPortal and releases the camera.
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
