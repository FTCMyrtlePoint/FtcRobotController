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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

public class RobotHardware2025M_10274 {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor redDrive   = null;
    private DcMotor blueDrive  = null;
    private DcMotor whiteDrive   = null;
    private DcMotor blackDrive  = null;

    private DcMotor Spin1   = null;
    private DcMotor rampMotor  = null;

    private DcMotor groundMotor   = null;
    private DcMotor midMotor  = null;

    private Servo Hand = null;
    private Servo Hold = null;

    public double Holdoffset       =  0 ;
    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double Hand_SERVO       =  0.5 ;
    public static final double Hold_SERVO       =  0.5 ;
    //public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection[] currentDetections;






    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware2025M_10274(LinearOpMode opmode) {
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
        redDrive  = myOpMode.hardwareMap.get(DcMotor.class, "BackLft");
        blueDrive = myOpMode.hardwareMap.get(DcMotor.class, "FrntRt");
        whiteDrive  = myOpMode.hardwareMap.get(DcMotor.class, "BackRt");
        blackDrive = myOpMode.hardwareMap.get(DcMotor.class, "FrntLft");


        Spin1  = myOpMode.hardwareMap.get(DcMotor.class, "spin1");

        rampMotor = myOpMode.hardwareMap.get(DcMotor.class, "ramp");

        groundMotor  = myOpMode.hardwareMap.get(DcMotor.class, "ground");
        midMotor = myOpMode.hardwareMap.get(DcMotor.class, "mid");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        redDrive.setDirection(DcMotor.Direction.FORWARD);
        blueDrive.setDirection(DcMotor.Direction.FORWARD);
        whiteDrive.setDirection(DcMotor.Direction.FORWARD);
        blackDrive.setDirection(DcMotor.Direction.FORWARD);

        Spin1.setDirection(DcMotor.Direction.REVERSE);

        rampMotor.setDirection(DcMotor.Direction.FORWARD);

        groundMotor.setDirection(DcMotor.Direction.FORWARD);
        midMotor.setDirection(DcMotor.Direction.FORWARD);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy

        redDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blueDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        whiteDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Spin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //groundMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //midMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        Hand = myOpMode.hardwareMap.get(Servo.class, "hand");


        //rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        Hand.setPosition(Hand_SERVO);
        //rightHand.setPosition(MID_SERVO);

        Hold = myOpMode.hardwareMap.get(Servo.class, "hold");

        Hold.setPosition(Hold_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Front     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Side      Right/Left turning power (-1.0 to 1.0) +ve is CW
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Front, double Side,double Turn) {
        // Combine drive and turn for blended motion.
        double FtLft  = Front + Side + Turn;
        double FtRgt = -Front + Side + Turn;
        double BkLft  = Front - Side + Turn;
        double BkRgt = -Front - Side + Turn;

        FtLft = Range.clip(FtLft, -1, 1);
        FtRgt = Range.clip(FtRgt, -1, 1);
        BkLft = Range.clip(BkLft, -1, 1);
        BkRgt = Range.clip(BkRgt, -1, 1);

        // Use existing function to drive both wheels.
        setDrivePower(FtRgt, FtLft, BkRgt, BkLft);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param FrontRight     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param FrontLeft    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param BackRight     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param BackLeft    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double FrontRight, double FrontLeft, double BackRight, double BackLeft) {
        // Output the values to the motor drives.
        blackDrive.setPower(FrontLeft);
        blueDrive.setPower(FrontRight);
        redDrive.setPower(BackLeft);
        whiteDrive.setPower(BackRight);
    }

    public void SetSpinPower(double power) {
        // Output the values to the motor drives.
        Spin1.setPower(power);
    }

    /*public void SetRampPower(double power) {
        // Output the values to the motor drives.
        rampMotor.setPower(power);
    }

    public void SetRampMotors(double power1, double power2) {
        // Output the values to the motor drives.
        groundMotor.setPower(power1);
        midMotor.setPower(power2);

    }*/

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, .4);
        Hand.setPosition(Hand_SERVO + offset);
        //rightHand.setPosition(MID_SERVO - offset);
    }

    public void setHoldPositions(boolean loaded) {
       if(loaded)
           Holdoffset = 0;
       else
           Holdoffset=.4;

        Hold.setPosition(Hold_SERVO + Holdoffset);
        //rightHand.setPosition(MID_SERVO - offset);
    }

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        myOpMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                myOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        myOpMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        myOpMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        myOpMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
        myOpMode.telemetry.update();

    }
    public void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        }
        else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }
}
