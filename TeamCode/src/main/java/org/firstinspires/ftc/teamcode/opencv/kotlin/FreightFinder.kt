package org.firstinspires.ftc.teamcode.opencv.kotlin

import org.apache.commons.math3.fraction.Fraction
import org.opencv.core.*
import org.openftc.easyopencv.OpenCvPipeline
import org.opencv.imgproc.Imgproc
import java.util.ArrayList

class FreightFinder(private var fov: Double, private var cameraPitchOffset: Double, private var cameraYawOffset: Double) : OpenCvPipeline() {
    val boxArray: MutableList<Rect> = mutableListOf()
    val ballArray: MutableList<Rect> = mutableListOf()

    val whiteContours: MutableList<MatOfPoint> = mutableListOf()
    val yellowContours: MutableList<MatOfPoint> = mutableListOf()

    var closestBox: Rect = Rect()
    var closestBall: Rect = Rect()
    var closestElement: Rect = Rect()

    protected var imageWidth = 0
    protected var imageHeight = 0

    private var horizontalFocalLength = 0.0
    private var verticalFocalLength = 0.0

    override fun init(mat: Mat) {
        super.init(mat)
        imageWidth = mat.width()
        imageHeight = mat.height()

        // pinhole model calculations
        val diagonalView = Math.toRadians(this.fov)
        val aspectFraction = Fraction(this.imageWidth, this.imageHeight)
        val horizontalRatio: Int = aspectFraction.numerator
        val verticalRatio: Int = aspectFraction.denominator
        val diagonalAspect = Math.hypot(horizontalRatio.toDouble(), verticalRatio.toDouble())
        val horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2
        val verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2))
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2))
    }

    override fun processFrame(input: Mat): Mat {
        val white = Mat()
        val orange = Mat()

        val lowWhite = Scalar(0.0, 0.0, 200.0) // wiffleball lower
        val highWhite = Scalar(180.0, 10.0, 255.0) // wiffleball upper
        val lowOrange = Scalar(14.0, 130.0, 80.0) // block lower
        val highOrange = Scalar(25.0, 255.0, 255.0) // block upper

        Imgproc.cvtColor(input, white, Imgproc.COLOR_RGB2HSV)
        Imgproc.cvtColor(input, orange, Imgproc.COLOR_RGB2HSV)

        Core.inRange(white, lowWhite, highWhite, white)
        Core.inRange(orange, lowOrange, highOrange, orange)


        Imgproc.GaussianBlur(white, white, Size(5.0, 5.0), 0.0, 0.0)
        Imgproc.GaussianBlur(orange, orange, Size(5.0, 5.0), 0.0, 0.0)



        val hierarchy1 = Mat()
        val hierarchy2 = Mat()

        whiteContours.clear()
        yellowContours.clear()
        Imgproc.findContours(white, whiteContours, hierarchy1, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE)
        Imgproc.findContours(orange, yellowContours, hierarchy2, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE)

        Imgproc.drawContours(input, whiteContours, -1, Scalar(0.0, 255.0, 0.0), 3)
        Imgproc.drawContours(input, yellowContours, -1, Scalar(0.0, 0.0, 255.0), 3)

        ballArray.clear();
        var highestBallY = 0;
        closestBall = Rect()
        for (contour in whiteContours) {
            val rect = Imgproc.boundingRect(contour)
            ballArray.add(rect)
            if(rect.y > highestBallY && rect.width > 10 && rect.height > 10) {
                highestBallY = rect.y
                closestBall = rect
            }
        }

        boxArray.clear();
        var highestBoxY = 0;
        closestBox = Rect()
        for (contour in yellowContours) {
            val rect = Imgproc.boundingRect(contour)
            boxArray.add(rect)
            if(rect.y > highestBoxY && rect.width > 10 && rect.height > 10) {
                highestBoxY = rect.y
                closestBox = rect
            }
        }

        if(highestBallY > highestBoxY) {
            closestElement = closestBall
        } else {
            closestElement = closestBox
        }


        orange.release()
        white.release()
        return input
    }

    fun calculateYaw(offsetCenterX: Double): Double? {
        return Math.atan((closestElement.x - offsetCenterX) / horizontalFocalLength)
    }
}