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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear OpMode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, select this sample, and select TeleOp.
 *  Also add another new file named RobotHardware.java, select the sample with that name, and select Not an OpMode.
 */

@TeleOp(name="OpModeB2025_10274", group="Robot")

public class Hardware_OpMode2025M_10274 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware2025M_10274 robot       = new RobotHardware2025M_10274(this);
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection[] currentDetections;

    @Override
    public void runOpMode() {
        double front        = 0;
        double turn         = 0;
        double side        = 0;
        double rampP        =0;
        double SpinPower   =-.75;
        //double RampPower = -1;
        boolean spin       = false;

        double ground        = 0;
        double middle = 0;
        double Loader1_offset=0;
        double Loader2_offset=0;
        double Fire_offset=0;
        boolean inlet=false;
        boolean fire = true;
        boolean firePM= true;

        int a=1;
        double speed=.07;

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1091.52, 1091.52, 424.814, 286.607)
                .build();


        robot.initAprilTag();





        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        robot.driveRobot(front, side, turn);
        robot.SetSpinPower(0);

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            if(gamepad1.left_bumper)
                a=-1;
            if(gamepad1.right_bumper)
                a=1;

            front = a*gamepad1.left_stick_y;
            side = a*-gamepad1.left_stick_x;
            turn = -1* gamepad1.right_stick_x;


            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(front, side, turn);


            //ground = -gamepad2.left_stick_y;
            //middle = -gamepad2.right_stick_y;






            if(gamepad1.x && spin)
                spin=false;
            if(gamepad1.y && !spin)
                spin=true;




            if(gamepad2.left_bumper && spin){
                robot.SetSpinPower(-0.85);
            }
            else if(spin){
                robot.SetSpinPower(SpinPower);
            }
            else{
                if(gamepad2.right_trigger>.1){
                    robot.SetSpinPower(-gamepad2.right_trigger);
                }
                else{
                    robot.SetSpinPower(0);
                }
            }





            if(gamepad2.a)
                inlet=true;

            if(gamepad2.b)
                inlet=false;




            if(gamepad2.y && fire)
                fire=false;
            if(!fire){
                inlet=false;
                if(firePM)
                    Fire_offset+=speed;
                if(Fire_offset>.99)
                    firePM=false;
            }
            if(!firePM){
                Fire_offset-=speed;
                if(Fire_offset<.01) {
                    fire = true;
                    firePM = true;
                    inlet=true;
                }
            }
            if(inlet){
                robot.setLoader1Positions(-1);
            }
            else{

                if(gamepad2.x){
                    robot.setLoader1Positions(1);
                }
                else{
                    robot.setLoader1Positions(0);
                }

                    
            }


            robot.setFirePositions(Fire_offset);





            if(gamepad2.dpad_down)
                Loader1_offset=0;











            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Front", "front");
            telemetry.addData("Side", "Right Stick");
            telemetry.addData("Turn", "Right Stick");
            robot.telemetryAprilTag();
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.update();

            }
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}