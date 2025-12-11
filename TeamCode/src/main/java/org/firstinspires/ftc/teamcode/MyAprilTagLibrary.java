package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

    public class MyAprilTagLibrary {

        RobotCamera_10274 robot  = new RobotCamera_10274();
        public static final AprilTagLibrary MY_TAG_LIBRARY;

        static {

            MY_TAG_LIBRARY = new AprilTagLibrary.Builder()
                    .addTag(20,"Blue_Tag",6.5, DistanceUnit.INCH)
                    .addTag(21, "Obelisk_GPP",6.5, DistanceUnit.INCH)
                    .addTag(22, "Obelisk_PGP",6.5, DistanceUnit.INCH)
                    .addTag(23,"Obelisk_PPG",6.5, DistanceUnit.INCH)
                    .addTag(24,"Red_Tag",6.5, DistanceUnit.INCH)
                    .build();
        }
    }