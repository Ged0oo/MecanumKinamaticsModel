import math
from wpimath.kinematics import (
    ChassisSpeeds,
    MecanumDriveKinematics,
    MecanumDriveOdometry,
    MecanumDriveWheelPositions,
    MecanumDriveWheelSpeeds,
)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


class MecanumDriveRobot:
    def __init__(self, gyro, initial_pose=Pose2d(0.0, 0.0, Rotation2d())):
        self.gyro = gyro
        self.pose = initial_pose

        # Define wheel locations (in meters) relative to the robot center
        self.wheel_locations = {
            "frontLeft": Translation2d(0.15, 0.15),
            "frontRight": Translation2d(0.15, -0.15),
            "backLeft": Translation2d(-0.15, 0.15),
            "backRight": Translation2d(-0.15, -0.15),
        }

        # Initialize kinematics
        self.kinematics = MecanumDriveKinematics(
            self.wheel_locations["frontLeft"],
            self.wheel_locations["frontRight"],
            self.wheel_locations["backLeft"],
            self.wheel_locations["backRight"],
        )

        # Initialize odometry with zero wheel positions
        self.odometry = MecanumDriveOdometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            MecanumDriveWheelPositions(),
            initial_pose,
        )

    def update_wheel_speeds(self, vx, vy, omega):
        """
        Updates the wheel speeds using the desired chassis speeds.
        	param vx: Forward speed (m/s)
        	param vy: Sideways speed (m/s)
        	param omega: Rotational speed (rad/s)
        """
        # Convert chassis speeds to wheel speeds
        chassis_speeds = ChassisSpeeds(vx, vy, omega)
        wheel_speeds = self.kinematics.toWheelSpeeds(chassis_speeds)
        return wheel_speeds

    def periodic(self):
        """ Update odometry and perform any necessary periodic tasks. """
        gyro_angle = self.gyro.getRotation2d()
        wheel_positions = self.get_wheel_positions()
        self.pose = self.odometry.update(gyro_angle, wheel_positions)
        print(f"Current Pose: {self.pose}")

    def get_wheel_positions(self):
        """ Placeholder for getting actual encoder readings. """
        # For now, return dummy encoder values (to be replaced with real ones)
        return MecanumDriveWheelPositions()


# Mock class for Gyro
class MockGyro:
    def getRotation2d(self):
        return Rotation2d.fromDegrees(0.0)  # Static gyro for simplicity


# Main control loop
def main():
    gyro = MockGyro()
    robot = MecanumDriveRobot(gyro)

    while True:
        try:
            vx = float(input("Enter forward speed (vx): "))  # Example input
            vy = float(input("Enter sideways speed (vy): "))  # Example input
            omega = float(input("Enter rotation speed (omega): "))  # Example input

            # Update wheel speeds based on user input
            wheel_speeds = robot.update_wheel_speeds(vx, vy, omega)
            print(f"Computed Wheel Speeds: {wheel_speeds}")

            # Perform periodic odometry update
            robot.periodic()

        except KeyboardInterrupt:
            print("Exiting the program.")
            break  # Exit the loop
        except Exception as e:
            print("An error occurred:", e)


if __name__ == "__main__":
    main()