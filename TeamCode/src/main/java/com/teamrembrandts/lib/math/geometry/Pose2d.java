package com.teamrembrandts.lib.math.geometry;

public class Pose2d {
    private Translation2d translation;
    private Rotation2d rotation;

    public Pose2d(Translation2d translation, Rotation2d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    public Pose2d add(Pose2d otherPose) {
        Translation2d newTranslation = translation.add(otherPose.getTranslation());
        Rotation2d newRotation = new Rotation2d(
                rotation.getRadians() + otherPose.getRotation().getRadians()
        );

        return new Pose2d(newTranslation, newRotation);
    }
}
