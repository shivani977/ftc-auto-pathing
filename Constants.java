package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.90);
    // change the number to mass of robot in kg
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotWidth(15.73)//put in robot width
            .robotLength(17) //put in robot length
            //.forwardTicksToInches(0.00793903)
            //.strafeTicksToInches(0.059768195281467686)
            //.forwardTicksToInches(-0.04153714)
            //.strafeTicksToInches(-0.020969)
            //.turnTicksToInches(0.02189169003275755);
            //.forwardTicksToInches(0.008011231705266077)

            //.forwardTicksToInches(0.009048123761196403)
            .forwardTicksToInches(0.00792)
            .strafeTicksToInches(0.00665)
            .turnTicksToInches(0.01804);

    //.strafeTicksToInches(0.00616089)
            //.turnTicksToInches(0.01510045583514629);
    //1.11186052




    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(361.28795892384915)
            //xVelocity(108.38638768)
            /*First, make sure you have enough room.
            By default, the robot moves 48 inches forward, but this can be changed by navigating to the ForwardVelocityTuner class in Tuning.java.
            Typically larger numbers yield better results.
            Then, in the Tuning OpMode, under automatic, select and start Forward Velocity Tuner.
            The robot speed should ramp up until it reaches full power.
            It will continue moving until it has reached the set distance, then it will abruptly stop.




            Once the robot stops moving at maximum speed, one number will be displayed on telemetry:


            Velocity: The final velocity the robot achieved before stopping; this is what we want


             */
            .yVelocity(165.99602112801944);
    //.yVelocity(49.79880634);
           /*


           First, make sure you have enough room. By default, the robot moves 48 inches to the left, but this can be changed by navigating to the LateralVelocityTuner class in Tuning.java. Typically larger numbers yield better results. Then, in the Tuning OpMode, under automatic, select and start Lateral Velocity Tuner. The robot speed should ramp up until it reaches full power. It will continue moving until it has reached the set distance, then it will abruptly stop.


           Once the robot stops moving at maximum speed, one number will be displayed on telemetry:


           Velocity: The final velocity the robot achieved before stopping; this is what we want


            */
    //.forwardZeroPowerAcceleration(deceleration)
    //.lateralZeroPowerAcceleration(deceleration)


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

