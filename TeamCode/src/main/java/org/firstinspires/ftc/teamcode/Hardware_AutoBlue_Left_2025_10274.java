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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="AutoModeBlue_Left_2025_10274", group="Robot")

public class Hardware_AutoBlue_Left_2025_10274 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware2025_10274 robot       = new RobotHardware2025_10274(this);

    private ElapsedTime     runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        double front        = 0;
        double turn         = 0;
        double side        = 0;
        double rampP        =0;
        double SpinPower   =-1;
        //double RampPower = -1;
        boolean spin       = false;
        boolean ramp = false;
        double ground        = 0;
        double middle = 0;
        double hand_offset=0;



        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        robot.driveRobot(front, side, turn);
        robot.SetSpinPower(0);
        //robot.SetRampPower(0);
        //robot.SetRampMotors(ground, middle);
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.



            front = 0;
            side = 0;
            turn = 0;
            robot.SetSpinPower(0);
            hand_offset = 0;
            robot.setHandPositions(hand_offset);

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(front, side, turn);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin right for 1.3 seconds
            front = 0;
            side = -.5;
            turn = 0;
            robot.driveRobot(front, side, turn);
            robot.SetSpinPower(0);
            hand_offset = 0;
            robot.setHandPositions(hand_offset);


            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin right for 1.3 seconds
            front = .5;
            side = 0;
            turn = 0;
            robot.driveRobot(front, side, turn);
            robot.SetSpinPower(0);
            hand_offset = 0;
            robot.setHandPositions(hand_offset);



            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin right for 1.3 seconds
            front = 0;
            side = 0;
            turn = .25;
            robot.driveRobot(front, side, turn);
            robot.SetSpinPower(0);
            hand_offset = 0;
            robot.setHandPositions(hand_offset);


            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin right for 1.3 seconds
            front = 0;
            side = 0;
            turn = 0;
            robot.driveRobot(front, side, turn);
            robot.SetSpinPower(0);
            hand_offset = 0;
            robot.setHandPositions(hand_offset);


            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 30)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin right for 1.3 seconds
            front = 0;
            side = 0;
            turn = 0;
            robot.driveRobot(front, side, turn);
            robot.SetSpinPower(0);
            hand_offset = 0;
            robot.setHandPositions(hand_offset);


            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Front", "front");
            telemetry.addData("Side", "Right Stick");
            telemetry.addData("Turn", "Right Stick");

            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
