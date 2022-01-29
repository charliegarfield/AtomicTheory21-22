package org.firstinspires.ftc.teamcode.mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Carousel implements Mechanism {
    int colorMultiplier;
    ElapsedTime timer = new ElapsedTime();
    public static PIDCoefficients coeffs = new PIDCoefficients(8, 0, 0);
    public double currentVelocity = 0;
    public double targetVelocity = 0;
    public double velocityError = 0;
    public Carousel(Color color) {
        if (color == Color.RED) {
            colorMultiplier = -1;
        } else if (color == Color.BLUE) {
            colorMultiplier = 1;
        }
    }
    public DcMotorEx carouselMotor;
    boolean aWasDown = false;
    boolean bWasDown = false;
    public static double maxVelocity = 2228.96;
    public static double maxAcceleration = 50;
    // Jerk isn't used if it's 0, but it might end up being necessary
    public static double maxJerk = 0;
    double oldMaxVelocity = maxVelocity;
    double oldMaxAcceleration = maxAcceleration;
    double oldMaxJerk = maxJerk;

    MotionProfile profile;
    MotionProfile negativeProfile;

    public void init(HardwareMap hardwareMap) {
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel");
        profile = generateMotionProfile(2500 * colorMultiplier);
        negativeProfile = generateMotionProfile(-2500 * colorMultiplier);
        // carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void run(Gamepad gamepad, boolean stickyMode) {
//        //This makes sure things only happen once.
//        if (gamepad.a) {
//            if (!aWasDown) {
//                // turnCarousel();
//                carouselMotor.setPower(colorMultiplier * 0.45);
//                aWasDown = true;
//                bWasDown = false;
//            }
//        } else if (gamepad.b) {
//            if (!bWasDown) {
//                // turnCarousel();
//                carouselMotor.setPower(-colorMultiplier * 0.45);
//                aWasDown = false;
//                bWasDown = true;
//            }
//        } else {
//            aWasDown = false;
//            bWasDown = false;
//            carouselMotor.setPower(0);
//        }
        if (gamepad.a) {
            if (!aWasDown) {
                // turnCarousel();
//                carouselMotor.setPower(colorMultiplier * 0.45);
                aWasDown = true;
                bWasDown = false;
                timer.reset();
            }
            followMotionProfile(profile);
        } else if (gamepad.b) {
            if (!bWasDown) {
                // turnCarousel();
//                carouselMotor.setPower(-colorMultiplier * 0.45);
                aWasDown = false;
                bWasDown = true;
                timer.reset();
            }
            followMotionProfile(negativeProfile);
        } else {
            aWasDown = false;
            bWasDown = false;
            carouselMotor.setPower(0);
        }

        if (maxVelocity != oldMaxVelocity || maxAcceleration != oldMaxAcceleration || maxJerk != oldMaxJerk) {
            profile = generateMotionProfile(2500 * colorMultiplier);
            negativeProfile = generateMotionProfile(-2500 * colorMultiplier);
        }

        oldMaxVelocity = maxVelocity;
        oldMaxAcceleration = maxAcceleration;
        oldMaxJerk = maxJerk;
    }

    public boolean turnCarousel() {
//        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        carouselMotor.setTargetPosition(colorMultiplier * 2500);
//        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        carouselMotor.setPower(0.45);
        return followMotionProfile(profile);
    }

    MotionProfile generateMotionProfile(double ticks) {
        if (ticks == 0){
            return null;
        }
        // Based on 60RPM motor, adjust if different
        return MotionProfileGenerator.generateSimpleMotionProfile(
        new MotionState(0, 0, 0),
        new MotionState(ticks, 0, 0),
        maxVelocity,
        maxAcceleration,
        maxJerk);
    }

    boolean followMotionProfile(MotionProfile profile){
        // specify coefficients/gains
// create the controller
        PIDFController controller = new PIDFController(coeffs);
        MotionState state = profile.get(timer.time());
        if (!state.equals(new MotionState(0, 0)) || timer.time() < 0.1) {
            controller.setTargetPosition(state.getX());
            controller.setTargetVelocity(state.getV());
            controller.setTargetAcceleration(state.getA());
            currentVelocity = carouselMotor.getVelocity();
            targetVelocity = state.getV();
            velocityError = state.getV() - carouselMotor.getVelocity();
// in each iteration of the control loop
// measure the position or output variable
// apply the correction to the input variable
            double correction = controller.update(carouselMotor.getCurrentPosition());
            carouselMotor.setPower(correction);
            return false;
        } else {
            timer.reset();
            return true;
        }
    }
}
