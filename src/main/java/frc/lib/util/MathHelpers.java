package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

public class MathHelpers {

    public static double reverseInterpolate(Translation2d query, Translation2d start, Translation2d end) {
        Translation2d segment = end.minus(start);
        Translation2d queryToStart = query.minus(start);

        double segmentLengthSqr = segment.getX() * segment.getX() + segment.getY() * segment.getY();

        if (segmentLengthSqr == 0.0) { // start and end are the same point
            return 0.0;
        }

        return dotProduct(queryToStart, segment) / segmentLengthSqr;
    }

    public static double distanceToLineSegment(Translation2d query, Translation2d start, Translation2d end) {
        double t = reverseInterpolate(query, start, end);
        if (t < 0.0) { // closest point is before start
            return query.getDistance(start);
        } else if (t > 1.0) { // closest point is after end
            return query.getDistance(end);
        } else { // closest point is within the segment
            Translation2d segment = end.minus(start);
            Translation2d closestPoint = start.plus(segment.times(t));
            return query.getDistance(closestPoint);
        }
    }

    public static double perpendicularDistanceToLine(Translation2d query, Translation2d start, Translation2d end) {
        double t = reverseInterpolate(query, start, end);
        Translation2d segment = end.minus(start);
        Translation2d closestPoint = start.plus(segment.times(t));
        return query.getDistance(closestPoint);
    }

    public static double distanceToPointInDirection(Pose2d first, Translation2d second) {
        return distanceToPointInDirection(first.getTranslation(), second, first.getRotation());
    }

    public static double distanceToPointInDirection(Translation2d first, Translation2d second, Rotation2d direction) {
        return dotProduct(second.minus(first), new Translation2d(direction.getCos(), direction.getSin()));
    }

    public static double dotProduct(Translation2d first, Translation2d second) {
        return first.getX() * second.getX() + first.getY() * second.getY();
    }

    public static double dotProduct(Translation3d first, Translation3d second) {
        return first.getX() * second.getX() + first.getY() * second.getY() + first.getZ() * second.getZ();
    }

    public static double angleBetween(Translation2d first, Translation2d second) {
        double dot = dotProduct(first, second);
        double norms = first.getNorm() * second.getNorm();
        if (norms == 0) return 0;
        return Math.acos(Math.max(-1.0, Math.min(1.0, dot / norms)));
    }

    public static double angleBetween(Translation3d first, Translation3d second) {
        double dot = dotProduct(first, second);
        double norms = first.getNorm() * second.getNorm();
        if (norms == 0) return 0;
        return Math.acos(Math.max(-1.0, Math.min(1.0, dot / norms)));
    }

    public static Translation2d projectOnto(Translation2d toProject, Translation2d projectionTarget) {
        double normSq = projectionTarget.getNorm() * projectionTarget.getNorm();
        if (normSq == 0) return Translation2d.kZero;
        return projectionTarget.times(dotProduct(toProject, projectionTarget) / normSq);
    }

    public static Translation3d projectOnto(Translation3d toProject, Translation3d projectionTarget) {
        double normSq = projectionTarget.getNorm() * projectionTarget.getNorm();
        if (normSq == 0) return Translation3d.kZero;
        return projectionTarget.times(dotProduct(toProject, projectionTarget) / normSq);
    }

    public static int crossProductDirection(Translation2d first, Translation2d second) {
        // useful for calculating flipping signs for some oriented actions
        double cross = first.getX() * second.getY() - first.getY() * second.getX();
        if (cross > 0) return 1;
        if (cross < 0) return -1;
        return 0;
    }

    public static Translation3d crossProduct(Translation3d first, Translation3d second) {
        return new Translation3d(
                first.getY() * second.getZ() - first.getZ() * second.getY(),
                first.getZ() * second.getX() - first.getX() * second.getZ(),
                first.getX() * second.getY() - first.getY() * second.getX());
    }

    public static Translation2d crossProductWithRotation(Translation2d translation2d, double rotation) {
        return new Translation2d(translation2d.getY() * rotation, translation2d.getX() * rotation);
    }

    public static Pose2d getParallelOffsetPose(Pose2d pose, double offsetMeters) {
        Translation2d offsetTranslation = pose.getTranslation()
                .plus(new Translation2d(
                        // Add 90 degrees to all trig functions
                        // so it is offset parallel to the face of the tag
                        -offsetMeters * pose.getRotation().getSin(),
                        offsetMeters * pose.getRotation().getCos()));

        return new Pose2d(offsetTranslation, pose.getRotation());

        // But this way simpler to reason about & understand (but wrong for rotated poses)
        // return pose.transformBy(new Transform2d(0, offsetMeters, Rotation2d.kZero));

    }

    public static Pose2d getPerpendicularOffsetPose(Pose2d pose, double perpendicularOffsetMeters) {
        Translation2d offsetTranslation = pose.getTranslation()
                .plus(new Translation2d(
                        perpendicularOffsetMeters * pose.getRotation().getCos(),
                        perpendicularOffsetMeters * pose.getRotation().getSin()));

        return new Pose2d(offsetTranslation, pose.getRotation());

        // But this way simpler to reason about & understand (but wrong for rotated poses)
        // return pose.transformBy(new Transform2d(perpendicularOffsetMeters, 0, Rotation2d.kZero));

    }

    public static Pose2d getOffsetPose(Pose2d pose, double parallelOffsetMeters, double perpendicularOffsetMeters) {
        return getPerpendicularOffsetPose(getParallelOffsetPose(pose, parallelOffsetMeters), perpendicularOffsetMeters);
    }

    public static Pose2d getOffsetPose(Pose2d pose, double distance, Rotation2d direction) {
        return pose.transformBy(
                new Transform2d(distance * direction.getCos(), distance * direction.getSin(), Rotation2d.kZero));
    }

    /**
     * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
     */
    public static double getParallelError(Pose2d origin, Pose2d target) {
        Translation2d originToTarget = origin.minus(target).getTranslation();
        Rotation2d angleBetween = originToTarget.getAngle();
        double parallelError = originToTarget.getNorm() * angleBetween.getSin();

        return parallelError;

        // return -origin.minus(target).getY(); (but wrong for rotated poses)
    }

    /**
     * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
     */
    public static double getPerpendicularError(Pose2d origin, Pose2d target) {
        Translation2d originToTarget = origin.minus(target).getTranslation();
        Rotation2d angleBetween = originToTarget.getAngle();
        double perpendicularError = originToTarget.getNorm() * angleBetween.getCos();

        return perpendicularError;

        // return -origin.minus(target).getX(); (but wrong for rotated poses)
    }

    /**
     * Checks if two line segments intersect.
     *
     * @param p1 Start of first segment
     * @param p2 End of first segment
     * @param q1 Start of second segment
     * @param q2 End of second segment
     * @return Optional containing intersection point if segments intersect, empty otherwise
     */
    public static Optional<Translation2d> lineSegmentIntersection(
            Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {

        double a1 = p2.getY() - p1.getY();
        double b1 = p1.getX() - p2.getX();
        double c1 = a1 * p1.getX() + b1 * p1.getY();

        double a2 = q2.getY() - q1.getY();
        double b2 = q1.getX() - q2.getX();
        double c2 = a2 * q1.getX() + b2 * q1.getY();

        double determinant = a1 * b2 - a2 * b1;

        if (Math.abs(determinant) < 1e-9) {
            // Lines are parallel or collinear
            if (Math.abs(a1 * q1.getX() + b1 * q1.getY() - c1) < 1e-9) {
                // Collinear: check for overlap
                double minX = Math.max(Math.min(p1.getX(), p2.getX()), Math.min(q1.getX(), q2.getX()));
                double maxX = Math.min(Math.max(p1.getX(), p2.getX()), Math.max(q1.getX(), q2.getX()));
                double minY = Math.max(Math.min(p1.getY(), p2.getY()), Math.min(q1.getY(), q2.getY()));
                double maxY = Math.min(Math.max(p1.getY(), p2.getY()), Math.max(q1.getY(), q2.getY()));

                if (minX <= maxX && minY <= maxY) {
                    // There is overlap, return midpoint of overlapping segment
                    double midX = (minX + maxX) / 2.0;
                    double midY = (minY + maxY) / 2.0;
                    return Optional.of(new Translation2d(midX, midY));
                }
            }
            return Optional.empty(); // Parallel but not overlapping
        }

        // Compute intersection point
        double x = (b2 * c1 - b1 * c2) / determinant;
        double y = (a1 * c2 - a2 * c1) / determinant;
        Translation2d intersection = new Translation2d(x, y);

        // Check if intersection is within both segments
        if (isPointOnSegmentForLineSegmentIntersection(intersection, p1, p2)
                && isPointOnSegmentForLineSegmentIntersection(intersection, q1, q2)) {
            return Optional.of(intersection);
        } else {
            return Optional.empty();
        }
    }

    private static boolean isPointOnSegmentForLineSegmentIntersection(
            Translation2d pt, Translation2d segStart, Translation2d segEnd) {
        double minX = Math.min(segStart.getX(), segEnd.getX()) - 1e-9;
        double maxX = Math.max(segStart.getX(), segEnd.getX()) + 1e-9;
        double minY = Math.min(segStart.getY(), segEnd.getY()) - 1e-9;
        double maxY = Math.max(segStart.getY(), segEnd.getY()) + 1e-9;

        return pt.getX() >= minX && pt.getX() <= maxX && pt.getY() >= minY && pt.getY() <= maxY;
    }

    /**
     * Returns true if two line segments intersect (including collinear overlaps).
     *
     * @param p1 Start of first segment
     * @param p2 End of first segment
     * @param q1 Start of second segment
     * @param q2 End of second segment
     * @return true if segments intersect
     */
    public static boolean doLineSegmentsIntersect(
            Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {

        // General case: check CCW
        if (ccwForDoLineSegmentsIntersect(p1, q1, q2) != ccwForDoLineSegmentsIntersect(p2, q1, q2)
                && ccwForDoLineSegmentsIntersect(p1, p2, q1) != ccwForDoLineSegmentsIntersect(p1, p2, q2)) {
            return true;
        }

        // Special case: collinear
        if (isPointOnSegmentForDoLineSegmentsIntersect(p1, q1, q2)) return true;
        if (isPointOnSegmentForDoLineSegmentsIntersect(p2, q1, q2)) return true;
        if (isPointOnSegmentForDoLineSegmentsIntersect(q1, p1, p2)) return true;
        if (isPointOnSegmentForDoLineSegmentsIntersect(q2, p1, p2)) return true;

        return false;
    }

    /** Checks if points a, b, c are in counter-clockwise order. */
    private static boolean ccwForDoLineSegmentsIntersect(Translation2d a, Translation2d b, Translation2d c) {
        return (c.getY() - a.getY()) * (b.getX() - a.getX()) > (b.getY() - a.getY()) * (c.getX() - a.getX());
    }

    /** Returns true if point p is on the line segment ab. */
    private static boolean isPointOnSegmentForDoLineSegmentsIntersect(
            Translation2d p, Translation2d a, Translation2d b) {
        double minX = Math.min(a.getX(), b.getX()) - 1e-9;
        double maxX = Math.max(a.getX(), b.getX()) + 1e-9;
        double minY = Math.min(a.getY(), b.getY()) - 1e-9;
        double maxY = Math.max(a.getY(), b.getY()) + 1e-9;

        return p.getX() >= minX
                && p.getX() <= maxX
                && p.getY() >= minY
                && p.getY() <= maxY
                && Math.abs(crossForDoLineSegmentsIntersect(a, b, p)) < 1e-9;
    }

    /** Cross product of vectors AB and AP. */
    private static double crossForDoLineSegmentsIntersect(Translation2d a, Translation2d b, Translation2d p) {
        return (b.getX() - a.getX()) * (p.getY() - a.getY()) - (b.getY() - a.getY()) * (p.getX() - a.getX());
    }

    public static Translation2d unitVector(Translation2d translation) {
        return translation.div(translation.getNorm());
    }

    /**
     * Transforms field-relative ChassisSpeeds from one point to another on a rigid body.
     * Adds the tangential velocity component (ω × offset) rotated to field frame.
     *
     * @param velocity  Field-relative chassis speeds at the source point
     * @param offset    Robot-relative offset from source to destination point
     * @param rotation  Current robot heading (for rotating offset to field frame)
     * @return Field-relative chassis speeds at the destination point
     */
    public static ChassisSpeeds transformVelocity(ChassisSpeeds velocity, Translation2d offset, Rotation2d rotation) {
        return new ChassisSpeeds(
                velocity.vxMetersPerSecond
                        + velocity.omegaRadiansPerSecond
                                * (offset.getY() * rotation.getCos() - offset.getX() * rotation.getSin()),
                velocity.vyMetersPerSecond
                        + velocity.omegaRadiansPerSecond
                                * (offset.getX() * rotation.getCos() + offset.getY() * rotation.getSin()),
                velocity.omegaRadiansPerSecond);
    }
}
