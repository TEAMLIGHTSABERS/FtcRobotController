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

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.NewHardwareMap;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular O;pMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Config
@TeleOp(name="Tournament_TeleOp", group="Iterative Opmode")

public class Tournament_TeleOp extends OpMode
{

    /* Declare OpMode members. */
    NewHardwareMap robot= new NewHardwareMap(); // use the class created to define Robot hardware
    boolean a_pressed = false;
    boolean b_pressed = false;
    boolean x_pressed = false;
    boolean y_pressed = false;
    boolean bumper_pressed = false;
    boolean trigger_pressed = false;
    boolean ShooterOn = false;
    static final double     PI = Math.PI;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP 0.025
    static final double     WHEEL_DIAMETER_INCHES   = 3.78;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    static final double LiftPos = 0;
    static final double posOpen = 0.5;
    static final double posClose =0.9;
    static final double INCREMENT   = 0.05;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    double  position_one = 0;
    double  position_two = 0;
    double pressed = 0;
    int liftposition = 0;

    //Enum Variable to represent different states of the intake
    public enum IntState {
        DRIVE,
        OPEN,
        PICKUP,
        CLOSED
    }

    IntState intState;  //Variable to hold the current state of the intake.

    //Enum Variable to represnt different states fo the drone launcher
    public enum DroneState {
        START,
        LAUNCH,
        D_RESET
    }

    DroneState droneState; //Variable to hold the current state of the drone launcher.

    public ElapsedTime mStateTime = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime(); //timer for intake movements
    public ElapsedTime droneTimer = new ElapsedTime(); //timer for drone launcher movements

    int v_state = 0;

    boolean scoring = false;

    FtcDashboard dashboard;


    //double wPower = 0.0;
    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */

    @Override

    public void init() {
        RobotLog.d("LOGGING START");
        robot.init(hardwareMap);
        intakeTimer.reset();
        intState = IntState.DRIVE;
        droneTimer.reset();
        droneState = DroneState.START;

        dashboard = FtcDashboard.getInstance();
    }


    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */

    @Override
    public void start() {
        robot.claw_right.setPosition(robot.claw_right_Close);
        robot.claw_left.setPosition(robot.claw_left_Close);
        robot.shoulder_right.setPosition(robot.shoulder_right_Down);
        robot.shoulder_left.setPosition(robot.shoulder_left_Down);
        robot.wrist_right.setPosition(robot.wrist_right_Drive);
        robot.wrist_left.setPosition(robot.wrist_left_Drive);
        robot.drone.setPosition(0.4);
        //robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double power = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double left = Range.clip(power - turn, -1.0, 1.0);
        double right = Range.clip(power + turn, -1.0, 1.0);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        /*
        //Turbo to 100%
        if (gamepad1.x && !x_pressed) {
            right = Range.clip(right * 1.4, -1.0, 1.0);
            left = Range.clip(left * 1.4, -1.0, 1.0);
        }*/


//(Note: The joystick goes negative when pushed forwards, so negate it for robot to drive forwards.)
//Left Joystick Manipulates Left Motors
        robot.FmotorLeft.setPower(left + strafe);
        robot.BmotorLeft.setPower(left - strafe);
        robot.FmotorRight.setPower(right - strafe);
        robot.BmotorRight.setPower(right + strafe);


        // Lift Motor Control (right bumper: up, right trigger: down)
        if ((gamepad1.right_bumper) && (gamepad1.right_trigger) < 0.25) {

            robot.LiftMotor.setPower(0.8);   // Lift UP
            liftposition = robot.LiftMotor.getCurrentPosition();
            telemetry.addData("Lift Position", liftposition);
        } else if ((gamepad1.right_trigger) > 0.25 && (!gamepad1.right_bumper) && ((robot.touch.isPressed()))) {
            robot.LiftMotor.setPower(-0.8);  // Lift DOWN
            liftposition = robot.LiftMotor.getCurrentPosition();
            telemetry.addData("Lift Position", liftposition);
        } else {
            robot.LiftMotor.setPower(0.05);
        }

        // Reset Lift Encoder when the lift is down and touch sensor is activated
        if(!robot.touch.isPressed()){
            robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftposition = robot.LiftMotor.getCurrentPosition();
            telemetry.addData("Lift Position", liftposition);
        }

        // Button press for high powered lift for hanging (a button)
        if(gamepad1.a && !a_pressed && ((robot.touch.isPressed()))) {
            robot.LiftMotor.setPower(-1.0);  // Lift DOWN for hang
        }

        // Drone Launcher code (x button)
        /*if (gamepad1.x && !x_pressed) {
            robot.droneMotor.setVelocity(robot.droneVel);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            robot.drone.setPosition(0.2);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            robot.droneMotor.setVelocity(0.0);
            robot.drone.setPosition(0.4);
        }*/
        switch(droneState){
            case START:
                if(gamepad1.x && !x_pressed){
                    robot.droneMotor.setVelocity(robot.droneVel);
                    droneTimer.reset();
                    droneState = DroneState.LAUNCH;
                }
                break;
            case LAUNCH:
                if(droneTimer.milliseconds() >= 2000){
                    robot.drone.setPosition(0.2);
                    droneTimer.reset();
                    droneState = DroneState.D_RESET;
                }
                break;
            case D_RESET:
                if(droneTimer.milliseconds() >= 1000){
                    robot.droneMotor.setVelocity(0.0);
                    robot.drone.setPosition(0.4);
                    droneState = DroneState.START;
                }
                break;
        }

        //Close both fingers or open left finger only (left bumper)
        if(gamepad1.left_bumper && !bumper_pressed){
            if((robot.claw_right.getPosition()<(robot.claw_right_Open+0.05))&&(robot.claw_left.getPosition()>(robot.claw_left_Open-0.05))){ //close both fingers
                closeFingers();
            }else if ((robot.claw_left.getPosition()>(robot.claw_left_Close-0.05))){ //open left finger only
                robot.claw_left.setPosition(robot.claw_left_Open);
            }
            bumper_pressed = true;
        }

        //Open and close right finger only (left trigger)
        if((gamepad1.left_trigger) > 0.25 && !trigger_pressed){
            if(robot.claw_right.getPosition()<(robot.claw_right_Close)+0.05){ //open right finger only
                robot.claw_right.setPosition(robot.claw_right_Open);
            }else if(robot.claw_right.getPosition()<(robot.claw_right_Open+0.05)){ //close right finger only
                robot.claw_right.setPosition(robot.claw_right_Close);
            }
            trigger_pressed = true;
        }

        //Move shoulder and wrist between drive and scoring position (b button)
        if(gamepad1.b && !b_pressed) {
            if (robot.shoulder_right.getPosition()<(robot.shoulder_right_Up+0.05)) { //pickup position
                // wrist up and then shoulder down
                robot.wrist_right.setPosition(robot.wrist_right_Drive);
                robot.wrist_left.setPosition(robot.wrist_left_Drive);
                mStateTime.reset();
                robot.up = false;
            }else if(robot.shoulder_right.getPosition()>(robot.shoulder_right_Down-0.05)){ //scoring position
                // shoulder up and then wrist down
                robot.shoulder_right.setPosition(robot.shoulder_right_Up);
                robot.shoulder_left.setPosition(robot.shoulder_left_Up);
                mStateTime.reset();
                robot.up = true;
            }
            b_pressed = true;
        }

        if (mStateTime.time() >= 0.5 && b_pressed && !robot.up) {
            robot.shoulder_right.setPosition(robot.shoulder_right_Down);
            robot.shoulder_left.setPosition(robot.shoulder_left_Down);
            if (!gamepad1.b) b_pressed = false;
        } else if (mStateTime.time() >= 0.5 && b_pressed && robot.up) {
            robot.wrist_right.setPosition(robot.wrist_right_Score);
            robot.wrist_left.setPosition(robot.wrist_left_Score);
            if (!gamepad1.b) b_pressed = false;
        }

        //Move fingers down  and close to pick up pixels (y button)
        /*if(gamepad1.y && !y_pressed) {
            if((robot.claw_right.getPosition()<(robot.claw_right_Open+0.05))&&(robot.claw_left.getPosition()>(robot.claw_left_Open-0.05))) {
                robot.wrist_right.setPosition(robot.wrist_right_Pu);
                robot.wrist_left.setPosition(robot.wrist_left_Pu);
                delay(250);
                robot.claw_left.setPosition(robot.claw_left_Close);
                robot.claw_right.setPosition(robot.claw_right_Close);
                delay(250);
                robot.wrist_right.setPosition(robot.wrist_right_Drive);
                robot.wrist_left.setPosition(robot.wrist_left_Drive);
            }else if ((robot.claw_right.getPosition()<(robot.claw_right_Close)+0.05)&&(robot.claw_left.getPosition()>(robot.claw_left_Close-0.05))){
                robot.claw_left.setPosition(robot.claw_left_Open);
                robot.claw_right.setPosition(robot.claw_right_Open);
            }
            y_pressed = true;
        }*/
        switch(intState){
            case DRIVE:
                if(gamepad1.y && !y_pressed){
                    if((robot.claw_right.getPosition()<(robot.claw_right_Open+0.05))&&(robot.claw_left.getPosition()>(robot.claw_left_Open-0.05))) {
                        intState = IntState.OPEN;
                    }else if ((robot.claw_right.getPosition()<(robot.claw_right_Close)+0.05)&&(robot.claw_left.getPosition()>(robot.claw_left_Close-0.05))){
                        openFingers();
                    }
                    y_pressed = true;
                }
                break;
            case OPEN:
                wristPickUp();
                intakeTimer.reset();
                intState = IntState.PICKUP;
                break;
            case PICKUP:
                if(intakeTimer.milliseconds() >= 250){
                    closeFingers();
                    intakeTimer.reset();
                    intState = IntState.CLOSED;
                }
                break;
            case CLOSED:
                if(intakeTimer.milliseconds() >= 250){
                    wristDrive();
                    intState = IntState.DRIVE;
                }
                break;
        }

        if (!gamepad1.a) a_pressed = false;
        //if (!gamepad1.b) b_pressed = false;
        if (!gamepad1.x) x_pressed = false;
        if (!gamepad1.y) y_pressed = false;
        if (!gamepad1.left_bumper) bumper_pressed = false;
        if ((gamepad1.left_trigger)<0.25) trigger_pressed = false;

    }//loop end


    // Code to Run When Coach Hits STOP
    @Override
    public void stop()
    {
        telemetry.addData("Robot", "Stopped");
    }
    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */

    private void closeFingers(){
        robot.claw_left.setPosition(robot.claw_left_Close);
        robot.claw_right.setPosition(robot.claw_right_Close);
    }

    private void openFingers(){
        robot.claw_left.setPosition(robot.claw_left_Open);
        robot.claw_right.setPosition(robot.claw_right_Open);
    }

    private void wristPickUp(){
        robot.wrist_right.setPosition(robot.wrist_right_Pu);
        robot.wrist_left.setPosition(robot.wrist_left_Pu);
    }

    private void wristDrive(){
        robot.wrist_right.setPosition(robot.wrist_right_Drive);
        robot.wrist_left.setPosition(robot.wrist_left_Drive);
    }

    public void delay(long millisToDelay) {
        long loopStartTimeInMillis = System.currentTimeMillis();
        while(System.currentTimeMillis() < loopStartTimeInMillis+millisToDelay) {

        }
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.20,
                //0.30, 0.40, 0.50, 0.55, 0.60, 0.65, 0.7, 0.75, 0.75};
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.72, 0.85, 1.00 };
        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }

}