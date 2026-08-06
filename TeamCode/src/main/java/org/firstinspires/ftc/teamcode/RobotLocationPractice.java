package org.firstinspires.ftc.teamcode;

public class RobotLocationPractice {

    double angle;
    double x;

    double y;

    // constructor method
    public RobotLocationPractice(double angle) {
        this.angle = angle;
    }

    public double getHeading() {
        // normalize angle for robot heading to be between -180 and 180 degrees

        double angle = this.angle;
        while (angle > 180) {
            angle -= 360; // subtract until within normal range
        }

        while (angle <= -180) {
            angle += 360; // add until within normal range
        }

        return angle; //return normalized angle
    }

    public void turnRobot(double angleChange) {
        angle += angleChange; // add the change in angle to the current angle
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public double getAngle()  {
        return this.angle;
    }

    public void changeX(double changeAmount) {
        x += changeAmount;
    }
    public void setX(double x) {
        this.x = x;
    }
    public double getX() {
        return this.x;
    }
    public void changeY(double changeAmount) {
        y += changeAmount;
    }
    public void setY(double y) {
        this.y = y;
    }
    public double getY() {
        return this.y;
    }
}

