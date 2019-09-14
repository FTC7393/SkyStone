package ftc.evlib.vision.processors;

import org.opencv.core.Point;
import org.opencv.core.Scalar;

import ftc.evlib.vision.ImageUtil;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/26/16
 *
 * A particle (field object) that can be tracked
 */
public class Particle {
    public enum Color {
        RED(ImageUtil.RED),
        BLUE(ImageUtil.BLUE),
        UNKNOWN(ImageUtil.WHITE);

        Color(Scalar color) {
            this.color = color;
        }

        public final Scalar color;

        public static Color fromTeamColor(TeamColor teamColor) {
            switch (teamColor) {
                case RED:
                    return RED;
                case BLUE:
                    return BLUE;
                default:
                    return UNKNOWN;
            }
        }

        public TeamColor toTeamColor() {
            switch (this) {
                case RED:
                    return TeamColor.RED;
                case BLUE:
                    return TeamColor.BLUE;
                default:
                    return TeamColor.UNKNOWN;
            }
        }
    }

    private final Color color;
    private final Vector2D fieldPosition;

    public Particle(Point fieldPosition, Color color) {
        this(fieldPosition.x, fieldPosition.y, color);
    }

    public Particle(double fieldX, double fieldY, Color color) {
        this(new Vector2D(fieldX, fieldY), color);
    }

    public Particle(Vector2D fieldPosition, Color color) {
        this.fieldPosition = fieldPosition;
        this.color = color;
    }

    public Vector2D getFieldPosition() {
        return fieldPosition;
    }

    public Color getColor() {
        return color;
    }

    @Override
    public String toString() {
        return "(" + Math.round(fieldPosition.getX() * 10.0) / 10.0 + "," + Math.round(fieldPosition.getY() * 10.0) / 10.0 + "," + color.name().charAt(0) + ")";
    }

//    @Override
//    public int compareTo(@NonNull Particle another) {
//        return Double.compare(another.radius, this.radius); //largest first (descending)
////        return Double.compare(this.radius, another.radius); //smallest first (ascending)
//    }
}
