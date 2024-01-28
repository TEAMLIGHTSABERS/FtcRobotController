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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class NewHardwareMap
{

    public DcMotor FmotorRight;
    public DcMotor FmotorLeft;
    public DcMotor BmotorRight;
    public DcMotor BmotorLeft;
    public DcMotorEx droneMotor = null;
    public DcMotor LiftMotor = null;
    public TouchSensor touch;
    public DistanceSensor sensorRange;
    public Servo pixel;
    public Servo drone;
    public Servo claw_right;
    public Servo claw_left;
    public Servo shoulder_right;
    public Servo shoulder_left;
    public Servo wrist_right;
    public Servo wrist_left;

    //position of servos
    //Less is more down
    public final double shoulder_left_Down = 0.12;
    public final double shoulder_left_Up = 0.51;
    //More is more down
    public final double shoulder_right_Down = 0.82;
    public final double shoulder_right_Up = 0.49;
    //Less is more down
    public final double wrist_left_Pu = 0.77;//.8
    public final double wrist_left_Drive = 0.83;
    public final double wrist_left_Score = 0.12;
    //More is more down
    //Right is 2 less than necessary for 1.0
    public final double wrist_right_Pu = 0.10;//.11
    public final double wrist_right_Drive = 0.08;
    public final double wrist_right_Score = 0.86;

    public final double claw_left_Open = 0.24;
    public final double claw_left_Close = 0.13;

    public final double claw_right_Open = 0.68;
    public final double claw_right_Close = 0.77;

    public final double droneVel = 2500;

    public boolean up = true;
    public boolean hand_close = false;
    public boolean hand_open = false;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public NewHardwareMap(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and created the configuration file.
         */

        //MOTORS
        FmotorRight = hwMap.dcMotor.get("right_motor"); //right1Motor
        FmotorRight.setDirection(DcMotor.Direction.REVERSE);
        FmotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BmotorRight = hwMap.dcMotor.get("b_right_motor"); //right2Motor
        BmotorRight.setDirection(DcMotor.Direction.REVERSE);
        BmotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FmotorLeft = hwMap.dcMotor.get("left_motor"); //left1Motor
        //FmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        FmotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BmotorLeft = hwMap.dcMotor.get("b_left_motor"); //left2Motor
        //BmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        BmotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftMotor = hwMap.dcMotor.get("lift_motor");
        LiftMotor.setDirection(DcMotor.Direction.REVERSE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        droneMotor = hwMap.get(DcMotorEx.class,"drone_motor");
        //droneMotor.setDirection(DcMotor.Direction.REVERSE);
        droneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touch = hwMap.get(TouchSensor.class, "lift_limit");


        //SERVOS

        //pixel = hwMap.servo.get("pixel_servo");

        drone = hwMap.servo.get("drone_servo");

        claw_right = hwMap.servo.get("claw_right");

        claw_left = hwMap.servo.get("claw_left");

        shoulder_right = hwMap.servo.get("shoulder_right");

        shoulder_left = hwMap.servo.get("shoulder_left");

        wrist_right = hwMap.servo.get("wrist_right");

        wrist_left = hwMap.servo.get("wrist_left");

    }
}