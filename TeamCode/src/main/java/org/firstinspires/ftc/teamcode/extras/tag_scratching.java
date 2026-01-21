package org.firstinspires.ftc.teamcode.extras;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@TeleOp(name = "AprilTag PitchRollYaw", group = "ProtoTypes")
@Disabled
@Deprecated
public class tag_scratching extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // true = use webcam, false = use RC phone camera
    private static final boolean USE_WEBCAM = true;

    @Override
    public void runOpMode() {

        initAprilTag();

        telemetry.addLine("Camera ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // Example lens intrinsics (adjust if you calibrate your own camera):
                .setLensIntrinsics(445.085, 445.085, 326.262, 235.802)
                // Optional: specify camera position/orientation on the robot
                .setCameraPose(
                        new Position(DistanceUnit.INCH, 0, 0, 0, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0)
                )
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("Detected Tags", detections.size());

        for (AprilTagDetection detection : detections) {
            telemetry.addLine(String.format("ID %d", detection.id));
            telemetry.addData("X (in)", "%.1f", detection.ftcPose.x);
            telemetry.addData("Y (in)", "%.1f", detection.ftcPose.y);
            telemetry.addData("Z (in)", "%.1f", detection.ftcPose.z);
            telemetry.addData("Pitch (deg)", "%.1f", detection.ftcPose.pitch);
            telemetry.addData("Roll (deg)", "%.1f", detection.ftcPose.roll);
            telemetry.addData("Yaw (deg)", "%.1f", detection.ftcPose.yaw);
            telemetry.addLine("--------------------");
        }

        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up)");
        telemetry.addLine("PRY = Pitch, Roll, Yaw (deg)");
    }
}
