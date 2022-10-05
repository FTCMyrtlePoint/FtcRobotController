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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class HardwarePushbot10274
{
    /* Public OpMode members. */
    public DcMotor  RedFLDrive   = null;
    public DcMotor  WhiteFRDrive  = null;
    public DcMotor  BRDrive     = null;
    public DcMotor  BlueBLDrive     = null;
    public DcMotor  ArmLeft     = null;
    public DcMotor  ArmRight     = null;
    public CRServo  leftClaw    = null;
    public CRServo  rightClaw   = null;
    public DcMotor  Spinius     = null;

    public static final double SERVO_TWO       =  0 ;
    public static final double SERVO_ONE      =  0 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot10274(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        RedFLDrive = hwMap.get(DcMotor.class, "RFL");
        WhiteFRDrive = hwMap.get(DcMotor.class, "WFR");
        BRDrive   = hwMap.get(DcMotor.class, "BR");
        BlueBLDrive  = hwMap.get(DcMotor.class, "BBL");
        ArmRight   = hwMap.get(DcMotor.class, "ArmR");
        ArmLeft  = hwMap.get(DcMotor.class, "ArmL");
        Spinius  = hwMap.get(DcMotor.class, "Spinny");

        RedFLDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        WhiteFRDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        BRDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        BlueBLDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        ArmRight.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        ArmLeft.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        ArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RedFLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WhiteFRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlueBLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spinius.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        RedFLDrive.setPower(0);
        WhiteFRDrive.setPower(0);
        BRDrive.setPower(0);
        BlueBLDrive.setPower(0);
        ArmRight.setPower(0);
        ArmLeft.setPower(0);
        Spinius.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        RedFLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WhiteFRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BlueBLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Spinius.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftClaw  = hwMap.get(CRServo.class, "LC");
        rightClaw = hwMap.get(CRServo.class, "RC");
        leftClaw.setPower(SERVO_ONE);
        rightClaw.setPower(SERVO_TWO);
    }
 }

