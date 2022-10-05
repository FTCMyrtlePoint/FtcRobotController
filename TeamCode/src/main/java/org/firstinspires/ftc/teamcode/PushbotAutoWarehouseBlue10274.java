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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * FACES TOWARDS WAREHOUSE
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Warehouse Blue", group="Pushbot")

public class PushbotAutoWarehouseBlue10274 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot10274 robot   = new HardwarePushbot10274();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    double          clawTwoOffset      = 0;                       // Servo mid position
    double          clawOneOffset      = 0;
    double    CLAW_TWO_SPEED  ;    // sets rate to move servo
    double    CLAW_ONE_SPEED  ;



    @Override
    public void runOpMode() {
        double Red = 0;
        double Blue = 0;
        double White = 0;
        double Black = 0;
        double Arm = 0;
        double drivey = 0;
        double drivex = 0;
        double turn = 0;
        double spin = 0;

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        robot.RedFLDrive.setPower(Red);
        robot.WhiteFRDrive.setPower(White);
        robot.BlueBLDrive.setPower(Blue);
        robot.BRDrive.setPower(Black);
        robot.ArmRight.setPower(Arm);
        robot.ArmLeft.setPower(-Arm);
        robot.Spinius.setPower(spin);
//Drive forward into warehouse
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            // Combine drive and turn for blended motion.


            drivey = .8;
            drivex = 0;
            turn = 0;

            Red = -drivey - drivex + turn;
            White = drivey - drivex + turn;
            Blue = -drivey + drivex + turn;
            Black = drivey + drivex + turn;

            // Normalize the values so neither exceed +/- 1.0
            //max = Math.max(Math.abs(Red), Math.abs(Black));
            //max = Math.max(Math.abs(White), Math.abs(Blue));


            Red = Range.clip(Red, -1, 1);
            Black = Range.clip(Black, -1, 1);
            Blue = Range.clip(Blue, -1, 1);
            White = Range.clip(White, -1, 1);

            robot.RedFLDrive.setPower(Red);
            robot.WhiteFRDrive.setPower(White);
            robot.BlueBLDrive.setPower(Blue);
            robot.BRDrive.setPower(Black);
            robot.ArmRight.setPower(Arm);
            robot.ArmLeft.setPower(-Arm);
            robot.Spinius.setPower(spin);
        }
        Arm = 0;
        drivey = 0;
        drivex = 0;
        turn = 0;
        spin = 0;

        Red = -drivey - drivex + turn;
        White = drivey - drivex + turn;
        Blue = -drivey + drivex + turn;
        Black = drivey + drivex + turn;

        // Normalize the values so neither exceed +/- 1.0
        //max = Math.max(Math.abs(Red), Math.abs(Black));
        //max = Math.max(Math.abs(White), Math.abs(Blue));


        Red = Range.clip(Red, -1, 1);
        Black = Range.clip(Black, -1, 1);
        Blue = Range.clip(Blue, -1, 1);
        White = Range.clip(White, -1, 1);

        robot.RedFLDrive.setPower(Red);
        robot.WhiteFRDrive.setPower(White);
        robot.BlueBLDrive.setPower(Blue);
        robot.BRDrive.setPower(Black);
        robot.ArmRight.setPower(Arm);
        robot.ArmLeft.setPower(-Arm);
        robot.Spinius.setPower(spin);
//Drive to the side so you are out of the way
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            // Combine drive and turn for blended motion.


            drivey = 0;
            drivex = 0.6;
            turn = 0;

            Red = -drivey - drivex + turn;
            White = drivey - drivex + turn;
            Blue = -drivey + drivex + turn;
            Black = drivey + drivex + turn;

            // Normalize the values so neither exceed +/- 1.0
            //max = Math.max(Math.abs(Red), Math.abs(Black));
            //max = Math.max(Math.abs(White), Math.abs(Blue));


            Red = Range.clip(Red, -1, 1);
            Black = Range.clip(Black, -1, 1);
            Blue = Range.clip(Blue, -1, 1);
            White = Range.clip(White, -1, 1);

            robot.RedFLDrive.setPower(Red);
            robot.WhiteFRDrive.setPower(White);
            robot.BlueBLDrive.setPower(Blue);
            robot.BRDrive.setPower(Black);
            robot.ArmRight.setPower(Arm);
            robot.ArmLeft.setPower(-Arm);
            robot.Spinius.setPower(spin);
        }
        Arm = 0;
        drivey = 0;
        drivex = 0;
        turn = 0;
        spin = 0;

        Red = -drivey - drivex + turn;
        White = drivey - drivex + turn;
        Blue = -drivey + drivex + turn;
        Black = drivey + drivex + turn;

        // Normalize the values so neither exceed +/- 1.0
        //max = Math.max(Math.abs(Red), Math.abs(Black));
        //max = Math.max(Math.abs(White), Math.abs(Blue));


        Red = Range.clip(Red, -1, 1);
        Black = Range.clip(Black, -1, 1);
        Blue = Range.clip(Blue, -1, 1);
        White = Range.clip(White, -1, 1);

        robot.RedFLDrive.setPower(Red);
        robot.WhiteFRDrive.setPower(White);
        robot.BlueBLDrive.setPower(Blue);
        robot.BRDrive.setPower(Black);
        robot.ArmRight.setPower(Arm);
        robot.ArmLeft.setPower(-Arm);
        robot.Spinius.setPower(spin);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            // Combine drive and turn for blended motion.


            drivey = 0;
            drivex = 0;
            turn = 0;

            Red = -drivey - drivex + turn;
            White = drivey - drivex + turn;
            Blue = -drivey + drivex + turn;
            Black = drivey + drivex + turn;

            // Normalize the values so neither exceed +/- 1.0
            //max = Math.max(Math.abs(Red), Math.abs(Black));
            //max = Math.max(Math.abs(White), Math.abs(Blue));


            Red = Range.clip(Red, -1, 1);
            Black = Range.clip(Black, -1, 1);
            Blue = Range.clip(Blue, -1, 1);
            White = Range.clip(White, -1, 1);

            robot.RedFLDrive.setPower(Red);
            robot.WhiteFRDrive.setPower(White);
            robot.BlueBLDrive.setPower(Blue);
            robot.BRDrive.setPower(Black);
            robot.ArmRight.setPower(Arm);
            robot.ArmLeft.setPower(-Arm);
            robot.Spinius.setPower(spin);
        }
        Arm = 0;
        drivey = 0;
        drivex = 0;
        turn = 0;
        spin = 0;

        Red = -drivey - drivex + turn;
        White = drivey - drivex + turn;
        Blue = -drivey + drivex + turn;
        Black = drivey + drivex + turn;

        // Normalize the values so neither exceed +/- 1.0
        //max = Math.max(Math.abs(Red), Math.abs(Black));
        //max = Math.max(Math.abs(White), Math.abs(Blue));


        Red = Range.clip(Red, -1, 1);
        Black = Range.clip(Black, -1, 1);
        Blue = Range.clip(Blue, -1, 1);
        White = Range.clip(White, -1, 1);

        robot.RedFLDrive.setPower(Red);
        robot.WhiteFRDrive.setPower(White);
        robot.BlueBLDrive.setPower(Blue);
        robot.BRDrive.setPower(Black);
        robot.ArmRight.setPower(Arm);
        robot.ArmLeft.setPower(-Arm);
        robot.Spinius.setPower(spin);


        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            // Combine drive and turn for blended motion.


            drivey = 0;
            drivex = 0;
            turn = 0;

            Red = -drivey - drivex + turn;
            White = drivey - drivex + turn;
            Blue = -drivey + drivex + turn;
            Black = drivey + drivex + turn;

            // Normalize the values so neither exceed +/- 1.0
            //max = Math.max(Math.abs(Red), Math.abs(Black));
            //max = Math.max(Math.abs(White), Math.abs(Blue));


            Red = Range.clip(Red, -1, 1);
            Black = Range.clip(Black, -1, 1);
            Blue = Range.clip(Blue, -1, 1);
            White = Range.clip(White, -1, 1);

            robot.RedFLDrive.setPower(Red);
            robot.WhiteFRDrive.setPower(White);
            robot.BlueBLDrive.setPower(Blue);
            robot.BRDrive.setPower(Black);
            robot.ArmRight.setPower(Arm);
            robot.ArmLeft.setPower(-Arm);
            robot.Spinius.setPower(spin);
        }
        Arm = 0;
        drivey = 0;
        drivex = 0;
        turn = 0;
        spin = 0;

        Red = -drivey - drivex + turn;
        White = drivey - drivex + turn;
        Blue = -drivey + drivex + turn;
        Black = drivey + drivex + turn;

        // Normalize the values so neither exceed +/- 1.0
        //max = Math.max(Math.abs(Red), Math.abs(Black));
        //max = Math.max(Math.abs(White), Math.abs(Blue));


        Red = Range.clip(Red, -1, 1);
        Black = Range.clip(Black, -1, 1);
        Blue = Range.clip(Blue, -1, 1);
        White = Range.clip(White, -1, 1);

        robot.RedFLDrive.setPower(Red);
        robot.WhiteFRDrive.setPower(White);
        robot.BlueBLDrive.setPower(Blue);
        robot.BRDrive.setPower(Black);
        robot.ArmRight.setPower(Arm);
        robot.ArmLeft.setPower(-Arm);
        robot.Spinius.setPower(spin);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
