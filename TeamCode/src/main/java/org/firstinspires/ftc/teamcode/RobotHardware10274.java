/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class RobotHardware10274 {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontWhite   = null;
    private DcMotor rightFrontRed  = null;
    private DcMotor leftBackBlue   = null;
    private DcMotor rightBackBlack = null;
    private DcMotor armMotor1 = null;
    private DcMotor armMotor2 = null;
    private CRServo wrist = null;
    private Servo   thumb = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double Thumb_Rest       =  0.5 ;
    public static final double Wrist_Rest       =  0 ;
    public static final double Wrist_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double Thumb_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware10274(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontWhite  = myOpMode.hardwareMap.get(DcMotor.class, "lefrWhite");
        rightFrontRed = myOpMode.hardwareMap.get(DcMotor.class, "rifrRed");
        leftBackBlue  = myOpMode.hardwareMap.get(DcMotor.class, "lebaBlue");
        rightBackBlack = myOpMode.hardwareMap.get(DcMotor.class, "ribaBlack");
        armMotor1   = myOpMode.hardwareMap.get(DcMotor.class, "arm1");
        armMotor2   = myOpMode.hardwareMap.get(DcMotor.class, "arm2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontWhite.setDirection(DcMotor.Direction.FORWARD);
        rightFrontRed.setDirection(DcMotor.Direction.FORWARD);
        leftBackBlue.setDirection(DcMotor.Direction.FORWARD);
        rightBackBlack.setDirection(DcMotor.Direction.FORWARD);



        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontWhite.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontRed.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackBlue.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackBlack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Motors resist movement under no power.
        leftFrontWhite.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontRed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackBlue.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackBlack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        wrist = myOpMode.hardwareMap.get(CRServo.class, "wrist");
        thumb = myOpMode.hardwareMap.get(Servo.class, "thumb");
        wrist.setPower(Wrist_Rest);
        thumb.setPosition(Thumb_Rest);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param frontBack     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param sideSide      Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left right hand turning power (-1.0 to 1.0) +ve is turn right
     */
    public void driveRobot(double frontBack, double sideSide, double turn) {
        // Combine drive and turn for blended motion.
        double a=1;
        double frLftWh  = a*frontBack - a*sideSide + a*turn;
        double frRghtRd = -a*frontBack - a*sideSide + a*turn;
        double BkLftBlu  = a*frontBack + a*sideSide + a*turn;
        double BkRghtBla = -a*frontBack + a*sideSide + a*turn;

        // Scale the values so neither exceed +/- 1.0
        frLftWh = Range.clip(frLftWh, -1, 1);
        frRghtRd = Range.clip(frRghtRd, -1, 1);
        BkLftBlu = Range.clip(BkLftBlu, -1, 1);
        BkRghtBla = Range.clip(BkRghtBla, -1, 1);

        // Use existing function to drive both wheels.
        setDrivePower(frLftWh, frRghtRd, BkLftBlu, BkRghtBla);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param LeftWhite     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param RightRed    Fwd/Rev driving power (-1.0 to 1.0) -ve is forward
     * @param LeftBlue     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param RightBlack    Fwd/Rev driving power (-1.0 to 1.0) -ve is forward
     */
    public void setDrivePower(double LeftWhite, double RightRed, double LeftBlue, double RightBlack) {
        // Output the values to the motor drives.
        leftFrontWhite.setPower(LeftWhite);
        rightFrontRed.setPower(RightRed);
        leftBackBlue.setPower(LeftBlue);
        rightBackBlack.setPower(RightBlack);
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        armMotor1.setPower(power);
        armMotor2.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offsetwrist
     * @param offsetthumb
     */
    public void setHandPositions(double offsetwrist, double offsetthumb) {
        offsetwrist = Range.clip(offsetwrist, -1, 1);
        wrist.setPower(Wrist_SPEED + offsetwrist);
        thumb.setPosition(Thumb_Rest - offsetthumb);
    }
}
