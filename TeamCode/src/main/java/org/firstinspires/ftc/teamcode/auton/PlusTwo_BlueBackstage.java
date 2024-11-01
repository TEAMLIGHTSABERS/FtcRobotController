/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MethodMap;
import org.firstinspires.ftc.teamcode.NewHardwareMap;
import org.firstinspires.ftc.teamcode.vision.CSVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@Autonomous(name="PlusTwo_BlueBackstage", preselectTeleOp = "Tournament_TeleOp")

public class PlusTwo_BlueBackstage extends LinearOpMode {
    //Put variables and classes here
    NewHardwareMap robot =   new NewHardwareMap();
    private double position = 0;

    /*
     * Variables used for switching cameras.
     */
    private WebcamName webcam1;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the OpenCV processor.
     */
    private CSVisionProcessor visionProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;

    /*private static int DESIRED_TAG_ID = -1; // Choose the tag you want to approach or set to -1 for ANY tag.

    private AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag

    private boolean targetFound = false;*/

    @Override
    public void runOpMode() {
        /**
         * Initialize the hardware map and the method map constructor into the opMode
         */

        robot.init(hardwareMap);
        MethodMap method = new MethodMap(this, robot, aprilTag, visionProcessor, myVisionPortal);
        robot.claw_right.setPosition(robot.claw_right_Close);
        robot.claw_left.setPosition(robot.claw_left_Close);
        robot.shoulder_right.setPosition(robot.shoulder_right_Down);
        robot.shoulder_left.setPosition(robot.shoulder_left_Down);
        robot.drone.setPosition(0.4);


        method.initDoubleVision();

        while (!isStopRequested() && opModeInInit()) {
            telemetry.addData("Identified", method.visionProcessor.getSelection());
            telemetry.update();
        }

        waitForStart();

        robot.wrist_right.setPosition(robot.wrist_right_Drive);
        robot.wrist_left.setPosition(robot.wrist_left_Drive);

        telemetry.addData("Identified", method.visionProcessor.getSelection());

        //Deliver the pixel to the correct spike mark
        method.myVisionPortal.setProcessorEnabled(method.visionProcessor, false);


            method.gyroDrive(0.4, 5, 0, 15.0);

            switch (method.visionProcessor.getSelection()) {
                case LEFT:
                    position = 1;
                    method.DESIRED_TAG_ID = 1;
                    method.pixelPos = -7;
                    method.gyroStrafe(0.6, 13, 0, 15.0);
                    method.gyroDrive(0.8, 13, 0, 15.0);
                    robot.claw_right.setPosition(robot.claw_right_Open);
                    method.gyroDrive(0.4, -5, 0, 15.0);
                    robot.claw_right.setPosition(robot.claw_right_Close);
                    method.gyroTurn(0.6, 90);
                    method.gyroDrive(0.4, 10, 90, 15.0);
                    method.gyroStrafe(0.6, -10, 90, 15.0);


                    break;
                case RIGHT:
                    method.DESIRED_TAG_ID = 3;
                    position = 3;
                    method.pixelPos = 5;
                    method.right_dis = 2;
                    method.gyroDrive(0.8, 22, 0, 15.0);
                    method.gyroTurn(0.6, -90);
                    method.gyroDrive(0.4, 2, -90, 15.0);
                    robot.claw_right.setPosition(robot.claw_right_Open);
                    method.gyroDrive(0.8, -7, -90, 15.0);
                    method.gyroTurn(0.6, 0);
                    robot.claw_right.setPosition(robot.claw_right_Close);
                    method.gyroTurn(0.6, 90);
                    method.gyroDrive(0.6, 20, 90, 15.0);
                    method.gyroStrafe(0.6, -10, 90, 15.0);

                    break;
                case MIDDLE:
                    method.DESIRED_TAG_ID = 2;
                    position = 2;
                    method.gyroStrafe(0.6, 8, 0, 15.0);
                    method.gyroDrive(0.8, 20, 0, 15.0);
                    //method.gyroTurn(0.6, -30);
                    //sleep(1000);
                    robot.claw_right.setPosition(robot.claw_right_Open);
                    method.gyroDrive(0.4, -5, 0, 15.0);
                    robot.claw_right.setPosition(robot.claw_right_Close);
                    method.gyroTurn(0.6, 90);
                    method.gyroDrive(0.4, 12, 90, 15.0);
                    method.gyroStrafe(0.6, -10, 90, 15.0);

            }

            robot.wrist_right.setPosition(robot.wrist_right_Drive);
            robot.wrist_left.setPosition(robot.wrist_left_Drive);

            //Drive to backdrop and deliver the yellow pixel to the correct spot

            while (!method.targetFound && opModeIsActive()) {

                List<AprilTagDetection> currentDetections = method.aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((method.DESIRED_TAG_ID < 0) || (detection.id == method.DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            method.targetFound = true;
                            method.desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }
            }

            robot.LiftMotor.setPower(1.0);
            sleep(350);
            robot.LiftMotor.setPower(0.05);

            robot.shoulder_right.setPosition(robot.shoulder_right_Up);
            robot.shoulder_left.setPosition(robot.shoulder_left_Up);
            sleep(500);
            robot.wrist_right.setPosition(robot.wrist_right_Score);
            robot.wrist_left.setPosition(robot.wrist_left_Score);
            sleep(200);

            method.gyroTurn(0.4, 90);

            method.gyroStrafe(0.5, -(method.desiredTag.ftcPose.x - method.DESIRED_DISTANCE_X), 90, 15.0);
            method.gyroDrive(0.5, method.desiredTag.ftcPose.y - method.DESIRED_DISTANCE + method.right_dis, 90, 15.0);

            //method.gyroDrive(0.5, 4, 90, 15.0);
            sleep(300);

            robot.claw_left.setPosition(robot.claw_left_Open);

            sleep(500);

            robot.LiftMotor.setPower(1.0);
            sleep(150);
            robot.LiftMotor.setPower(0.05);

            method.gyroDrive(0.6, -7, 90, 15.0);
            method.gyroTurn(0.4, 90);
            method.gyroStrafe(0.8, -27+method.pixelPos, 90, 15.0);

            robot.wrist_right.setPosition(robot.wrist_right_Drive);
            robot.wrist_left.setPosition(robot.wrist_left_Drive);
            sleep(500);
            robot.claw_right.setPosition(robot.claw_right_Close);
            robot.claw_left.setPosition(robot.claw_left_Close);
            sleep(500);
            robot.shoulder_right.setPosition(robot.shoulder_right_Down);
            robot.shoulder_left.setPosition(robot.shoulder_left_Down);
            sleep(500);
            while(robot.touch.isPressed()) {
                robot.LiftMotor.setPower(-0.8);
            }
            robot.LiftMotor.setPower(0);

            //method.gyroDrive(0.8, 7, 90, 15.0);
            method.gyroTurn(0.6, -90);
            //method.gyroDrive(0.8, -15, -90, 15.0);

           //Go for two pixels
            sleep(3000);
            method.DESIRED_TAG_ID = 9;
            method.gyroDrive(0.8, 87, -90, 15);
            method.gyroStrafe(0.8, 32, -90,15);

        while (!method.targetFound && opModeIsActive()) {

            List<AprilTagDetection> currentDetections = method.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((method.DESIRED_TAG_ID < 0) || (detection.id == method.DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        method.targetFound = true;
                        method.desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
        }

        method.gyroStrafe(0.5, -(method.desiredTag.ftcPose.x - method.DESIRED_DISTANCE_X), -90, 15.0);
        method.gyroDrive(0.5, method.desiredTag.ftcPose.y - method.DESIRED_DISTANCE, -90, 15.0);

        method.liftDrive(0.4, method.liftNum, 15.0);
        robot.claw_right.setPosition(0.62);
        method.gyroDrive(0.4, 7, 90, 15.0);
        robot.claw_right.setPosition(robot.claw_right_Close);
        sleep(200);
        method.gyroDrive(0.4, -7, 90, 15.0);

        

    }


    /**
     * Initialize AprilTag and TFOD.
     */
    /*private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.

        aprilTag.setDecimation(3);

        // -----------------------------------------------------------------------------------------
        // OpenCV Configuration
        // -----------------------------------------------------------------------------------------

        visionProcessor = new CSVisionProcessor();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------


        //Switched for speed, needs to be fixed!
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(visionProcessor, aprilTag)
                .build();

        }// end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /*
    Manually set the camera gain and exposure.
    This can only be called AFTER calling initAprilTag(), and only works for Webcams;
   */
    /*private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (myVisionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }*/

}


