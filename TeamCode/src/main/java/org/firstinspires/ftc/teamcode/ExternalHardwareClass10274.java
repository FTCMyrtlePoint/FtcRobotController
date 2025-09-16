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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;

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

@TeleOp(name="RobotTele10274", group="RobotHdw10274")

public class ExternalHardwareClass10274 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware10274 robot       = new RobotHardware10274(this);

    @Override
    public void runOpMode() {
        double front        = 0;
        double turn         = 0;
        double side        = 0;
        boolean flipit=true;

        int g=0;
        int p=20;
        double arm1          = 0;
        double arm2          = 0;
        double arm3         = 0;
        int armOffset       =0;
        int armOffset2       =0;
        int armSpeed        =25;
        int armSpeed2       =4;
        double handOffset   = 0;
        //double holdArmup = 0;
        //double holdArmdown = 0;
        //double holdoutArm = 0;
        //double holdinArm = 0;
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        robot.ResetArms();
        robot.setArmPower(arm1,arm2, arm3);
        //robot.setArmPower(arm1, arm2);
        robot.driveRobot(front, side, turn);
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            //if(flipit)
            //    if(gamepad1.left_bumper)
            //        flipit=false;

            if(!flipit)
                if(gamepad1.right_bumper)
                    flipit=true;

            if(flipit) {
                if (Math.abs(gamepad1.left_stick_y) > .1) {
                    front = gamepad1.left_stick_y;
                } else
                    front = 0;
                if (Math.abs(gamepad1.left_stick_x) > .1) {
                    side = -gamepad1.left_stick_x;
                } else
                    side = 0;
                if (Math.abs(gamepad1.right_stick_x) > .1) {
                    turn = -.75 * gamepad1.right_stick_x;
                } else
                    turn = 0;
            }
            else{
                if (Math.abs(gamepad1.left_stick_y) > .1) {
                    front = -gamepad1.left_stick_y;
                } else
                    front = 0;
                if (Math.abs(gamepad1.left_stick_x) > .1) {
                    side = gamepad1.left_stick_x;
                } else
                    side = 0;
                if (Math.abs(gamepad1.right_stick_x) > .1) {
                    turn = -.75* gamepad1.right_stick_x;
                } else
                    turn = 0;
            }
 
            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(front, side, turn);

            // Use gamepad left & right Bumpers to open and close the claw
            // Use the SERVO constants defined in RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range
            if (gamepad2.left_bumper)
               handOffset = -.8;
            else if (gamepad2.right_bumper)
                handOffset = 0;
            handOffset = Range.clip(handOffset, -0.9, .1);

            // Move both servos to new position.  Use RobotHardware class
            robot.setHandPositions(handOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in RobotHardware class.
            //arm1= gamepad2.left_stick_y;
            //arm2= gamepad2.right_stick_y;

            //robot.setArmPower(arm1, arm2);
            /*if(robot.armMotor.getCurrentPosition()>-400){

                arm1= .75*gamepad2.left_stick_y;
                armOffset = Range.clip(armOffset, -1025, -500);

                g=-1025;
                armOffset=-1025;
                arm2= .95;
                robot.setArmPositions(g,p,arm1,arm2);
            }*/

            if(gamepad1.y)
                arm3=.5;
            else
                arm3=0;

            if(gamepad1.left_bumper)
                arm2= -.5;
            else
                arm2=0;

            if(gamepad1.b)
                arm3=.5;
            else
                arm3=0;

            if(gamepad1.right_bumper)
                arm2= -.5;
            else
                arm2=0;

            //armOffset = Range.clip(armOffset, -1025, -500);
            //armOffset2 = Range.clip(armOffset2, -135, 16);
            //g=armOffset;
            //p=armOffset2;
            arm1= .5*gamepad1.left_trigger;
            arm1= .5*gamepad1.right_trigger;

            //if(gamepad2.left_trigger>.5)
            //    arm1=gamepad2.left_trigger;
            //if(gamepad2.right_trigger>.5)
            //    arm2=gamepad2.right_trigger;

            robot.setArmPower(arm1,arm2, arm3);




            //robot.setArmPositions(g,p,arm1,arm2);
            //robot.setArmPower(g, p);
            // Send telemetry messages to explain controls and show robot status
            //telemetry.addData("Front", "front");
            //telemetry.addData("Side", "Right Stick");
            //telemetry.addData("Turn", "Right Stick");
            //telemetry.addData("Arm Up/Down", "Y & A Buttons");
            //telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
            //telemetry.addData("-", "-------");
            //telemetry.addData("position1", robot.armMotor.getCurrentPosition());
            //telemetry.addData("position2", robot.armMotor2.getCurrentPosition());
            //telemetry.addData("Goposition1", g);
            //telemetry.addData("Goposition2", p);
            //telemetry.addData("FwdBk Power", "%.2f", front);
            //telemetry.addData("LftRgt Power",  "%.2f", side);
            //telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Arm1 Power",  "%.2f", arm1);
            telemetry.addData("Arm2 Power",  "%.2f", arm2);
            telemetry.addData("Arm3 Power",  "%.2f", arm3);
            //telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
