// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RangefinderConstants;
import frc.robot.subsystems.Rangefinder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AddWallToField extends Command {
  
  private final Rangefinder m_rangefinder;
  private final Field2d m_field;
  private final Supplier<ChassisSpeeds> m_fieldChassisSupplier;
  private final double MERGE_DISTANCE = 0.12;
  private static final double CLUSTER_DISTANCE = 0.35;

  private final List<WallCluster> m_wallClusters = new ArrayList<>();
  
  /**
   * A stabilized wall point.
   *
   * <p>Stores both the averaged position and how many
   * measurements contributed to it.
   */
  private static class WallPoint {

    public Translation2d point;

    public int samples;

    public WallPoint(Translation2d point) {
      this.point = point;
      this.samples = 1;
    }
  }

  /**
   * A disconnected wall segment displayed as its own
   * FieldObject2d.
   */
  private static class WallCluster {

    public final List<WallPoint> points =
        new ArrayList<>();

    public final FieldObject2d fieldObject;

    public WallCluster(FieldObject2d fieldObject) {
      this.fieldObject = fieldObject;
    }
  }



  /**
   * Creates a new wall mapping command.
   *
   * @param fieldRelitiveChassisSpeeds field-relative chassis speeds
   * @param field field widget
   * @param rangefinder rangefinder subsystem
   */
  public AddWallToField(
      Supplier<ChassisSpeeds> fieldRelitiveChassisSpeeds,
      Field2d field,
      Rangefinder rangefinder) {

    m_rangefinder = rangefinder;
    m_field = field;
    m_fieldChassisSupplier =
        fieldRelitiveChassisSpeeds;

    addRequirements(rangefinder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double meters = m_rangefinder.getDistanceMeters();

    if (!isMeasurementValid(meters)) {
      return;
    }

    if (isRotatingTooFast()) {
      return;
    }

    Translation2d wallPoint = calculateWallPoint(meters);

    addPointToMap(wallPoint);

    updateFieldDisplay();
  }

  /**
   * Determines whether a sensor measurement is valid.
   *
   * @param meters measured distance
   * @return true if usable
   */
  private boolean isMeasurementValid(double meters) {
    return meters > RangefinderConstants.kMinDistance
        && meters < RangefinderConstants.kMaxDistance;
  }

  /**
   * Determines whether the robot is rotating too quickly
   * for reliable wall mapping.
   *
   * @return true if rotating too fast
   */
  private boolean isRotatingTooFast() {
    return Math.abs(m_fieldChassisSupplier.get().omegaRadiansPerSecond) > RangefinderConstants.kMaxUseableAngularSpeed;
  }

  /**
   * Converts a rangefinder measurement into a field point.
   *
   * @param meters measured sensor distance
   * @return detected wall point in field coordinates
   */
  private Translation2d calculateWallPoint(double meters) {
    Pose2d robotPose =m_field.getRobotPose();
    Pose2d rangefinderPose =robotPose.transformBy(RangefinderConstants.kRangeFinderOffset);
    Pose2d wallPose = rangefinderPose.transformBy(new Transform2d(meters,0,new Rotation2d()));
    return wallPose.getTranslation();
  }

  /**
   * Adds a detected point into the wall map.
   *
   * <p>The point is either:
   *
   * <ul>
   *   <li>Merged into an existing wall point
   *   <li>Added to an existing wall cluster
   *   <li>Used to create a new wall cluster
   * </ul>
   *
   * @param newPoint detected wall point
   */
  private void addPointToMap(Translation2d newPoint) {

    WallCluster nearestCluster = findNearestCluster(newPoint);

    if (nearestCluster == null) {

      createNewCluster(newPoint);

      return;
    }

    mergeIntoCluster(nearestCluster,newPoint);
  }

  /**
   * Finds the nearest wall cluster to a point.
   *
   * @param point point to compare
   * @return nearest cluster or null if none nearby
   */
  private WallCluster findNearestCluster(Translation2d point) {

    WallCluster closestCluster = null;
    double closestDistance = Double.MAX_VALUE;
    for (WallCluster cluster : m_wallClusters) {
      for (WallPoint wallPoint : cluster.points) {
        double distance = wallPoint.point.getDistance(point);
        if (distance < closestDistance) {
          closestDistance = distance;
          closestCluster = cluster;
        }
      }
    }

    if (closestDistance > CLUSTER_DISTANCE) {
      return null;
    }

    return closestCluster;
  }

  /**
   * Merges a point into a cluster or adds it as a new
   * point inside the cluster.
   *
   * @param cluster target cluster
   * @param newPoint incoming point
   */
  private void mergeIntoCluster(WallCluster cluster,Translation2d newPoint) {
    for (WallPoint wallPoint : cluster.points) {
      double distance = wallPoint.point.getDistance(newPoint);
      if (distance < MERGE_DISTANCE) {
        weightedAverage(wallPoint,newPoint);
        return;
      }
    }

    cluster.points.add(new WallPoint(newPoint));
  }

  /**
   * Updates a wall point using weighted averaging.
   *
   * <p>Points with many samples become more stable and
   * resistant to noise.
   *
   * @param wallPoint existing wall point
   * @param newPoint incoming measurement
   */
  private void weightedAverage(WallPoint wallPoint,Translation2d newPoint) {

    int samples = wallPoint.samples;
    double averagedX = (wallPoint.point.getX() * samples + newPoint.getX()) / (samples + 1);
    double averagedY = (wallPoint.point.getY() * samples + newPoint.getY()) / (samples + 1);
    wallPoint.point = new Translation2d(averagedX, averagedY);
    wallPoint.samples++;
  }

  /**
   * Creates a brand new wall cluster.
   *
   * @param firstPoint first point in the wall
   */
  private void createNewCluster(Translation2d firstPoint) {
    FieldObject2d object = m_field.getObject("Wall " + m_wallClusters.size());
    WallCluster cluster = new WallCluster(object);
    cluster.points.add(new WallPoint(firstPoint));
    m_wallClusters.add(cluster);
  }

  /**
   * Updates all wall objects on the field widget.
   */
  private void updateFieldDisplay() {

    for (WallCluster cluster : m_wallClusters) {
      List<Pose2d> poses = new ArrayList<>();
      for (WallPoint point : cluster.points) {
        poses.add(new Pose2d(point.point,new Rotation2d()));
      }
      cluster.fieldObject.setPoses(poses);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}