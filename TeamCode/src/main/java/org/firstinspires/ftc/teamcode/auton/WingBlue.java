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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MethodMap;
import org.firstinspires.ftc.teamcode.NewHardwareMap;
import org.firstinspires.ftc.teamcode.vision.CSVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name="WingBlue", preselectTeleOp = "Tournament_TeleOp")

public class WingBlue extends LinearOpMode {
    //Put variables and classes here
    NewHardwareMap robot =   new NewHardwareMap();

    /*
     * Variables used for switching cameras.
     */
    private WebcamName webcam1, webcam2;

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

    private static int DESIRED_TAG_ID = -1; // Choose the tag you want to approach or set to -1 for ANY tag.

    private AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag

    private boolean targetFound = false;

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
        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch (method.visionProcessor.getSelection()) {
            case RIGHT:
                method.DESIRED_TAG_ID = 3;
                method.pixelPos = 8;
                method.left_strafe = 8;
                method.right_forward = 3;
                method.right_dis = 1;
                method.gyroStrafe(0.6, -15, 0, 15.0);
                method.gyroDrive(0.8, 13, 0, 15.0);
                robot.claw_left.setPosition(robot.claw_left_Open);
                method.gyroDrive(0.4, -5, 0, 15.0);
                robot.claw_left.setPosition(robot.claw_left_Close);
                method.gyroStrafe(0.6, -11, 0, 15.0);
                method.gyroDrive(0.4, 33, 0, 15.0);
                method.gyroStrafe(0.6, 7, 0, 15.0);
                method.gyroTurn(0.6, -90);
                method.liftDrive(0.4, method.liftNum, 15.0);
                robot.claw_left.setPosition(0.35);
                method.gyroDrive(0.4, 8, -90, 15.0);
                robot.claw_left.setPosition(robot.claw_left_Close);
                sleep(200);
                method.gyroDrive(0.4, -10, -90, 15.0);
                method.gyroTurn(0.6, 90);

                break;
            case LEFT:
                method.DESIRED_TAG_ID = 1;
                method.pixelPos = 3;
                method.right_forward = 3;
                method.right_dis = 2;
                method.left_strafe = -10;
                method.gyroDrive(0.8, 22, 0, 15.0);
                method.gyroTurn(0.6, 90);
                robot.claw_left.setPosition(robot.claw_left_Open);
                method.gyroDrive(0.8, -15, 90, 15.0);
                robot.claw_left.setPosition(robot.claw_left_Close);
                method.gyroTurn(0.6, -90);
                method.gyroStrafe(0.4, 6,-90, 15.0);
                method.liftDrive(0.4, method.liftNum, 15.0);
                robot.claw_left.setPosition(0.35);
                method.gyroDrive(0.4, 7, -90, 15.0);
                robot.claw_left.setPosition(robot.claw_left_Close);
                sleep(200);
                method.gyroDrive(0.4, -7, -90, 15.0);
                method.gyroTurn(0.6, -90);
                method.gyroStrafe(0.4, 21,-90, 15.0);
                method.gyroTurn(0.6, 90);

                break;
            case MIDDLE:
                method.DESIRED_TAG_ID = 2;
                method.right_dis = 2;
                method.pixelPos = 3;
                method.gyroStrafe(0.6, -8, 0, 15.0);
                method.gyroDrive(0.8, 20, 0, 15.0);
                robot.claw_left.setPosition(robot.claw_left_Open);
                method.gyroDrive(0.4, -5, 0, 15.0);
                robot.claw_left.setPosition(robot.claw_left_Close);
                method.gyroTurn(0.6, -90);
                sleep(200);
                method.gyroDrive(0.4, 12, -90, 15.0);
                method.gyroTurn(0.6, -90);
                method.gyroStrafe(0.6, 2, -90, 15.0);
                method.liftDrive(0.4, method.liftNum, 15.0);
                robot.claw_left.setPosition(0.35);
                method.gyroDrive(0.4, 7, -90, 15.0);
                robot.claw_left.setPosition(robot.claw_left_Close);
                sleep(200);
                method.gyroDrive(0.4, -7, -90, 15.0);
                method.gyroTurn(0.6, -90);
                method.gyroStrafe(0.6, 32, -90, 15.0);
                method.gyroTurn(0.6, 0);
                method.gyroTurn(0.6, 90);
                method.backdropStrafe = true;
        };

        robot.wrist_right.setPosition(robot.wrist_right_Drive_A);
        robot.wrist_left.setPosition(robot.wrist_left_Drive_A);
        while(robot.touch.isPressed() && opModeIsActive()) {
            robot.LiftMotor.setPower(-0.8);
        }
        robot.LiftMotor.setPower(0);

        sleep(2000); //Sleep however long is necessary for alliance partner to finish

        method.gyroTurn(0.6, 90);
        method.gyroDrive(0.8, 87-method.pixelPos, 90, 15.0);
        sleep(200);
        method.gyroStrafe(0.6, 27-method.left_strafe, 90, 15.0);
        method.gyroTurn(0.6, 90);

        //Drive to backdrop and deliver the yellow pixel to the correct spot

        while(!method.targetFound && opModeIsActive()) {

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
        sleep(450);
        robot.LiftMotor.setPower(0.05);

        robot.shoulder_right.setPosition(robot.shoulder_right_Up);
        robot.shoulder_left.setPosition(robot.shoulder_left_Up);
        sleep(500);
        robot.wrist_right.setPosition(robot.wrist_right_Score);
        robot.wrist_left.setPosition(robot.wrist_left_Score);
        sleep(200);

        method.gyroTurn(0.4, 90);

        method.gyroStrafe(0.5, -(method.desiredTag.ftcPose.x-method.DESIRED_DISTANCE_X-method.right_dis), 90, 15.0);
        method.gyroDrive(0.5, method.desiredTag.ftcPose.y-method.DESIRED_DISTANCE-method.right_forward+1, 90, 15.0);


        sleep(300);

        robot.claw_right.setPosition(robot.claw_right_Open);

        sleep(300);

        robot.LiftMotor.setPower(1.0);
        sleep(150);
        robot.LiftMotor.setPower(0.0);


        if (method.DESIRED_TAG_ID == 2) {
            method.gyroStrafe(0.6, 5, 90, 15.0);
        }
        else if (method.DESIRED_TAG_ID == 1){
            method.gyroStrafe(0.6, -10, 90, 15.0);
        }

        method.gyroTurn(0.4, 90);

        sleep(200);
        robot.claw_left.setPosition(robot.claw_left_Open);

        sleep(200);

        method.gyroDrive(0.6, -6, 90, 15.0);
        //method.gyroStrafe(0.8, -26 - method.right_dis, 90, 15.0);

        robot.wrist_right.setPosition(robot.wrist_right_Drive);
        robot.wrist_left.setPosition(robot.wrist_left_Drive);
        sleep(300);
        robot.claw_right.setPosition(robot.claw_right_Close);
        robot.claw_left.setPosition(robot.claw_left_Close);
        sleep(300);
        robot.shoulder_right.setPosition(robot.shoulder_right_Down);
        robot.shoulder_left.setPosition(robot.shoulder_left_Down);
        sleep(300);
        while(robot.touch.isPressed() && opModeIsActive()) {
            robot.LiftMotor.setPower(-0.8);
        }
        robot.LiftMotor.setPower(0);

        //method.gyroDrive(0.8, 7, -90, 15.0);
        method.gyroTurn(0.4, 0);
        method.gyroTurn(0.4, -90);

        sleep(2000);


        }

}


