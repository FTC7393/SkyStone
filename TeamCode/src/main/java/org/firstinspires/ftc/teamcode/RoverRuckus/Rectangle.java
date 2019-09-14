package org.firstinspires.ftc.teamcode.RoverRuckus;

public class Rectangle {
    private final int x1, x2, y1, y2;

    public Rectangle(int x1, int x2, int y1, int y2) {
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
    }
    public boolean isInside (int x, int y) {
        if(x > x1 && x < x2 && y > y1 && y < y2) {
            return true;
        }
        return false;
    }
}
