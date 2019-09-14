package ftc.evlib.vision.processors;

import com.google.common.collect.ImmutableList;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 1/11/17
 */

public class Particles extends ArrayList<Particle> {
    /**
     * The radius of a particle to be used in duplicate finding and pathfinding calculations
     */
    public static final Distance PARTICLE_RADIUS = Distance.fromInches(2);

    /**
     * The width of the robot for pathfinding. Might need to be changed to the diagonal of the robot
     */
    public static final Distance ROBOT_WIDTH = Distance.fromInches(18);
    public static final Distance ROBOT_LENGTH = Distance.fromInches(18);

    /**
     * Extra margin for for pathfinding
     */
    public static final Distance GAP_MARGIN = Distance.fromInches(2);

    /**
     * The minimum gap between the centers of two particles for the robot to fit through
     */
    public static final double WIDTH_GAP = ROBOT_WIDTH.inches() + PARTICLE_RADIUS.inches() * 2 + GAP_MARGIN.inches();
    public static final double LENGTH_GAP = ROBOT_LENGTH.inches() + PARTICLE_RADIUS.inches() * 2 + GAP_MARGIN.inches();

    /**
     * The distance to turn the collector on before the robot reaches the ball
     */
    public static final Distance TURN_ON_COLLECTOR_EARLY = Distance.fromInches(3);

    /**
     * The distance to drive past a particle when collecting it
     */
    public static final Distance EXTRA_DRIVE_TO_COLLECT = Distance.fromInches(3);

    /**
     * Add a particle when the phone/robot is turned at an angle
     *
     * @param particle the particle to add
     * @param angle    the angle the robot is turned at
     */
    public void add(ftc.evlib.vision.processors.Particle particle, Angle angle) {
        super.add(new ftc.evlib.vision.processors.Particle(
                new Vector2D(
                        particle.getFieldPosition().getLength(),
                        Angle.add(Angle.subtract(Angle.fromDegrees(90), particle.getFieldPosition().getDirection()), angle)
                ),
                particle.getColor()
        ));
//        Log.i("Particle", "added particle angle: " + particle.getFieldPosition().getDirection().degrees() + " degrees");
//        Log.i("Particle", "added particle angle2: " + Angle.add(particle.getFieldPosition().getDirection(), angle).degrees() + " degrees");
    }

    /**
     * Add multiple particles when the phone/robot is turned at an angle
     *
     * @param particles the particles to add
     * @param angle     the angle the robot is turned at
     */
    public void add(List<Particle> particles, Angle angle) {
        for (ftc.evlib.vision.processors.Particle particle : particles) {
            add(particle, angle);
        }
    }

    /**
     * combine particles that have the same color and overlap in space
     */
    //TODO average the positions of combined particles
    public void removeDuplicates() {
        Set<Integer> toRemove = new HashSet<>();
        int size = this.size();
        for (int i = 0; i < size - 1; i++) {
            ftc.evlib.vision.processors.Particle particle1 = this.get(i);
            Vector2D pos1 = particle1.getFieldPosition();
            ftc.evlib.vision.processors.Particle.Color color1 = particle1.getColor();
            for (int j = i + 1; j < size; j++) {
                if (i != j) {
                    ftc.evlib.vision.processors.Particle particle2 = this.get(j);
                    if (color1 == particle2.getColor()) {
                        Vector2D pos2 = particle2.getFieldPosition();
                        double dx = pos1.getX() - pos2.getX();
                        double dy = pos1.getY() - pos2.getY();
                        double distance = Math.sqrt(dx * dx + dy * dy);
                        if (distance < PARTICLE_RADIUS.inches() * 2) {
                            toRemove.add(j);
                        }
                    }
                }
            }
        }
        Integer[] removeArray = (Integer[]) toRemove.toArray();

        for (int i = removeArray.length - 1; i >= 0; i--) {
            this.remove(removeArray[i]);
        }
    }

    /**
     * Get the movements of the robot to collect a particular particle.
     * For all but the last movement, the collector should be turning backwards to expel particles.
     * For the last movement, the collector should move forward to collect the particle
     *
     * @param teamColor the color of particle to collect
     * @return a list of Vector2D objects that represents the movements in inches
     */
    public List<Vector2D> getMovementsToParticle(TeamColor teamColor) {
        return getMovementsToParticle(ftc.evlib.vision.processors.Particle.Color.fromTeamColor(teamColor));
    }

    /**
     * Get the movements of the robot to collect a particular particle.
     * For all but the last movement, the collector should be turning backwards to expel particles.
     * For the last movement, the collector should move forward to collect the particle
     *
     * @param particleColor the color of particle to collect
     * @return a list of Vector2D objects that represents the movements in inches
     */
    public List<Vector2D> getMovementsToParticle(ftc.evlib.vision.processors.Particle.Color particleColor) {
        Particles unobstructedParticles = getUnobstructedParticles(particleColor);
        if (unobstructedParticles.size() > 0) {
//            a ball of the desired color is NOT obstructed
            Vector2D nearest = unobstructedParticles.getNearestParticle().getFieldPosition();
//            make 1 long movement and stop right before the particle to turn the collector on
            double dist1 = nearest.getLength() - TURN_ON_COLLECTOR_EARLY.inches();
            if (dist1 < 0) dist1 = 0;
//            make one short movement to collect the particle
            double dist2 = TURN_ON_COLLECTOR_EARLY.inches() + EXTRA_DRIVE_TO_COLLECT.inches();

//            Log.i("Particle", "nearest particle angle" + nearest.getDirection().degrees() + " degrees");


            Vector2D movement1 = new Vector2D(dist1, nearest.getDirection());
            Vector2D movement2 = new Vector2D(dist2, nearest.getDirection());
            return ImmutableList.of(movement1, movement2);
        }

//        a ball of the desired color IS obstructed
//        Particles particles = getParticlesOfColor(particleColor);
        List<Particles> obstructionsList = getObstructionsForParticles(particleColor, true);
        for (Particles obstructions : obstructionsList) {
            Vector2D pos = obstructions.getFarthestParticle().getFieldPosition();
            double distToParticle = pos.getLength();
            Angle angleToParticle = pos.getDirection();
            Angle driveAngle1 = Angle.add(angleToParticle, Angle.fromRadians(Math.atan2(WIDTH_GAP / 2, distToParticle)));
            Angle driveAngle2 = Angle.subtract(angleToParticle, Angle.fromRadians(Math.atan2(WIDTH_GAP / 2, distToParticle)));

            //TODO select between adding and subtracting from the particle angle
            Angle driveAngle = driveAngle1;
            double arcLength = distToParticle + LENGTH_GAP;


            Vector2D movement0 = new Vector2D(arcLength, driveAngle);
            Particles unobstructedParticles1 = getUnobstructedParticlesAt(movement0, particleColor);

            if (unobstructedParticles1.size() > 0) {
//            a ball of the desired color is NOT obstructed from the second drive point
                Vector2D nearest = unobstructedParticles1.getNearestParticle().getFieldPosition();
                nearest = new Vector2D(
                        nearest.getX() - movement0.getX(),
                        nearest.getY() - movement0.getY()
                );
//            make 1 long movement and stop right before the particle to turn the collector on
                double dist1 = nearest.getLength() - TURN_ON_COLLECTOR_EARLY.inches();
                if (dist1 < 0) dist1 = 0;
//            make one short movement to collect the particle
                double dist2 = TURN_ON_COLLECTOR_EARLY.inches() + EXTRA_DRIVE_TO_COLLECT.inches();

//            Log.i("Particle", "nearest particle angle" + nearest.getDirection().degrees() + " degrees");

                Vector2D movement1 = new Vector2D(dist1, nearest.getDirection());
                Vector2D movement2 = new Vector2D(dist2, nearest.getDirection());
                return ImmutableList.of(movement0, movement1, movement2);
            }
        }


//        define an arc that brings the robot past the obstructing particles
//        find an angle on the arc that will not result in the particle or obstructing particls(s) being bumped
//        2 movements
        return ImmutableList.of();
    }

    public Particles getUnobstructedParticles(ftc.evlib.vision.processors.Particle.Color particleColor) {
        return getUnobstructedParticlesAt(new Vector2D(0, 0), particleColor);
    }

    /**
     * Get all the particles that are not obstructed from a certain point in field coordinates
     *
     * @param centerPoint   the point to measure from
     * @param particleColor the color of the requested particles
     * @return the list of unobstructed particles
     */
    public Particles getUnobstructedParticlesAt(Vector2D centerPoint, ftc.evlib.vision.processors.Particle.Color particleColor) {
        Particles particles = new Particles();
        for (int i = 0; i < this.size(); i++) {
            ftc.evlib.vision.processors.Particle particle = this.get(i);

            if (particle.getColor() == particleColor) {
                if (getObstructionsForParticleAt(i, centerPoint, false).size() == 0) {
                    particles.add(particle);
                }

            }
        }
        return particles;
    }


    private List<Particles> getObstructionsForParticles(ftc.evlib.vision.processors.Particle.Color particleColor, boolean getAll) {
        return getObstructionsForParticlesAt(particleColor, new Vector2D(0, 0), getAll);
    }

    /**
     * Get a list of Particle objects, each of which contains the obstructing particles for a particular particles
     *
     * @param centerPoint the robot's position
     * @param doGetAll    whether to get the first obstruction or all the obstructions
     * @return a list of Particles objects
     */
    private List<Particles> getObstructionsForParticlesAt(ftc.evlib.vision.processors.Particle.Color particleColor, Vector2D centerPoint, boolean doGetAll) {
        List<Particles> obstructions = new ArrayList<>();
        for (int i = 0; i < this.size(); i++) {
            if (get(i).getColor() == particleColor) {
                obstructions.add(getObstructionsForParticleAt(i, centerPoint, doGetAll));
            }
        }
        return obstructions;
    }

    /**
     * Get either the first obstruction found or all obstructing particles for a particular particle
     *
     * @param i           the index of the particle to find the obstructions for
     * @param centerPoint the position of the robot
     * @param doGetAll    whether or not to retrieve all the obstructions
     * @return a new Particles object with the obstructing particles
     */
    private Particles getObstructionsForParticleAt(int i, Vector2D centerPoint, boolean doGetAll) {
        Particles obstructions = new Particles();

        double cx = centerPoint.getX();
        double cy = centerPoint.getY();

        ftc.evlib.vision.processors.Particle particle = this.get(i);
        Vector2D position = new Vector2D(
                particle.getFieldPosition().getX() - cx,
                particle.getFieldPosition().getY() - cy
        );
        Angle direction = position.getDirection();
        double distance = position.getLength();
        for (int j = 0; j < this.size(); j++) {
            if (i != j) {
                ftc.evlib.vision.processors.Particle particle1 = this.get(j);

                Vector2D position1 = new Vector2D(
                        particle1.getFieldPosition().getX() - cx,
                        particle1.getFieldPosition().getY() - cy
                );
                Angle direction1 = position1.getDirection();
                double distance1 = position1.getLength();

                if (distance1 < distance) {
                    Angle robotAngle = Angle.fromRadians(Math.atan2(WIDTH_GAP / 2, distance1));
                    if (Math.abs(direction.radians() - direction1.radians()) <= Math.abs(robotAngle.radians())) {
                        //particle is in the way
                        obstructions.add(particle1);
                        if (!doGetAll) break;
                    }
                }
            }
        }
        return obstructions;
    }


    /**
     * Get the particle closest to the robot
     *
     * @return the closest particle
     */
    public ftc.evlib.vision.processors.Particle getNearestParticle() {
        double minDistance = Double.MAX_VALUE;
        ftc.evlib.vision.processors.Particle nearestParticle = null;
        for (ftc.evlib.vision.processors.Particle particle : this) {
            double distance = particle.getFieldPosition().getLength();
            if (distance < minDistance) {
                nearestParticle = particle;
                minDistance = distance;
            }
        }
        return nearestParticle;
    }

    /**
     * Get the particle furthest from the robot
     *
     * @return the closest particle
     */
    public ftc.evlib.vision.processors.Particle getFarthestParticle() {
        double minDistance = Double.MIN_VALUE;
        ftc.evlib.vision.processors.Particle nearestParticle = null;
        for (ftc.evlib.vision.processors.Particle particle : this) {
            double distance = particle.getFieldPosition().getLength();
            if (distance > minDistance) {
                nearestParticle = particle;
                minDistance = distance;
            }
        }
        return nearestParticle;
    }

    /**
     * Create a new particles list of the particles of just one color
     *
     * @param particleColor the color to be retrieved
     * @return a new Particles object
     */
    private Particles getParticlesOfColor(ftc.evlib.vision.processors.Particle.Color particleColor) {
        Particles particles = new Particles();
        for (ftc.evlib.vision.processors.Particle particle : this) {
            if (particle.getColor() == particleColor) {
                particles.add(particle);
            }
        }
        return particles;
    }
}
