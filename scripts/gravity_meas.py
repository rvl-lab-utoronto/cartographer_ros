import rosbag
import numpy as np

def process_imu_data(bag_file):
    with rosbag.Bag(bag_file, 'r') as bag:
        linear_accelerations = []
        angular_velocities = []

        for topic, msg, _ in bag.read_messages(topics=['/ouster/imu']):
            # Collect the linear accelerations
            linear_acc = msg.linear_acceleration
            linear_accelerations.append([linear_acc.x, linear_acc.y, linear_acc.z])

            # Collect the angular velocities
            angular_vel = msg.angular_velocity
            angular_velocities.append([angular_vel.x, angular_vel.y, angular_vel.z])

        # Convert to numpy arrays
        linear_accelerations = np.array(linear_accelerations)
        angular_velocities = np.array(angular_velocities)

        # Calculate covariance matrix of linear accelerations
        calculated_acc_covariances = np.cov(linear_accelerations, rowvar=False)

        # Calculate covariance matrix of angular velocities
        calculated_angular_covariances = np.cov(angular_velocities, rowvar=False)

        # Calculate mean linear acceleration for gravity estimation
        gravity_estimates = linear_accelerations.mean(axis=0)

        # Calculate a single gravity constant estimate
        gravity_constant_estimate = np.linalg.norm(gravity_estimates)

        return (calculated_acc_covariances, calculated_angular_covariances,
                gravity_estimates, gravity_constant_estimate)

bag_file = '/home/sepehr/Downloads/ouster_imu_robot_stationary_2025-04-30-08-58-17.bag'
(acc_covariances, angular_covariances, gravity_estimates, gravity_constant_estimate) = process_imu_data(bag_file)

print("Calculated Linear Acceleration Covariance Matrix:")
print(acc_covariances)
print("\nCalculated Angular Velocity Covariance Matrix:")
print(angular_covariances)
print("\nGravity Estimates (x, y, z):")
print(gravity_estimates)
print("\nGravity Constant Estimate:")
print(gravity_constant_estimate)