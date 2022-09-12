//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
//import org.firstinspires.ftc.teamcode.shplib.vision.opencv.AprilTagPipeline;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//// TODO
//// install .so file to control hub
//
//// sim can be downloaded at https://github.com/deltacv/EOCV-Sim/releases
//
//public class Vision extends Subsystem {
//    private final OpenCvCamera camera;
//    private final AprilTagPipeline aprilTagPipeline;
//
//    private final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    private final double fx = 578.272;
//    private final double fy = 578.272;
//    private final double cx = 402.145;
//    private final double cy = 221.506;
//
//    // UNITS ARE METERS, MUST CHANGE
//    private final double tagsize = 0.173;
//
//    private int numFramesWithoutDetection = 0;
//
//    private final float DECIMATION_HIGH = 3;
//    private final float DECIMATION_LOW = 2;
//    private final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
//    private final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
//
//    private ArrayList<AprilTagDetection> detections = new ArrayList<AprilTagDetection>();
//
//    public Vision(HardwareMap hardwareMap) {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
//            }
//        });
//    }
//
//    public ArrayList<AprilTagDetection> getDetections() {
//        return detections;
//    }
//
//    @Override
//    public void periodic(Telemetry telemetry) throws InterruptedException {
//        // Calling getDetectionsUpdate() will only return an object if there was a new frame
//        // processed since the last time we called it. Otherwise, it will return null. This
//        // enables us to only run logic when there has been a new frame, as opposed to the
//        // getLatestDetections() method which will always return an object.
//        ArrayList<AprilTagDetection> newDetections = aprilTagPipeline.getDetectionsUpdate();
//
//        if (newDetections != null) {
////            telemetry.addData("FPS", camera.getFps());
////            telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
////            telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
//
//            // If we don't see any tags
//            if (newDetections.size() == 0) {
//                numFramesWithoutDetection++;
//
//                // If we haven't seen a tag for a few frames, lower the decimation
//                // so we can hopefully pick one up if we're e.g. far back
//                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
//                    aprilTagPipeline.setDecimation(DECIMATION_LOW);
//                }
//            }
//            // We do see tags!
//            else {
//                numFramesWithoutDetection = 0;
//
//                // If the target is within 1 meter, turn on high decimation to
//                // increase the frame rate
//                if (newDetections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
//                    aprilTagPipeline.setDecimation(DECIMATION_HIGH);
//                }
//
//                detections = newDetections;
//
////                for (AprilTagDetection detection : detections) {
////                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
////                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
////                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
////                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
////                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
////                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
////                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
////                    telemetry.addData("center x: ", detection.center.x);
////                    telemetry.addData("center y: ", detection.center.y);
////                }
//            }
//        }
//        Thread.sleep(20);
//    }
//}
