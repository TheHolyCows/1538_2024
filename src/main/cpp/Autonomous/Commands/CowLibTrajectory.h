#pragma once

#include <units/time.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/curvature.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/MathUtil.h>
#include <vector>
#include <memory>
#include <optional>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/util/GeometryUtil.h>
#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

namespace pathplanner {

class CowLibTrajectory {
public:

	CowLibTrajectory() {
	}

	CowLibTrajectory(std::vector<PathPlannerTrajectory::State> states) : m_states(states) {
	}

	/**
	 * Generate a CowLibTrajectory
	 *
	 * @param path PathPlannerPath to generate the trajectory for
	 * @param startingSpeeds Starting speeds of the robot when starting the trajectory
	 * @param startingRotation Starting rotation of the robot when starting the trajectory
	 */
	CowLibTrajectory(std::shared_ptr<PathPlannerPath> path,
			const frc::ChassisSpeeds &startingSpeeds,
			const frc::Rotation2d &startingRotation) : m_states(
			generateStates(path, startingSpeeds, startingRotation)) {
	}

	/**
	 * Get the target state at the given point in time along the trajectory
	 *
	 * @param time The time to sample the trajectory at in seconds
	 * @return The target state
	 */
	PathPlannerTrajectory::State sample(const units::second_t time);

	/**
	 * Get all of the pre-generated states in the trajectory
	 *
	 * @return vector of all states
	 */
	constexpr std::vector<PathPlannerTrajectory::State>& getStates() {
		return m_states;
	}

	/**
	 * Get the total run time of the trajectory
	 *
	 * @return Total run time in seconds
	 */
	inline units::second_t getTotalTime() {
		return getEndState().time;
	}

	/**
	 * Get the goal state at the given index
	 *
	 * @param index Index of the state to get
	 * @return The state at the given index
	 */
	inline PathPlannerTrajectory::State getState(size_t index) {
		return m_states[index];
	}

	/**
	 * Get the initial state of the trajectory
	 *
	 * @return The initial state
	 */
	inline PathPlannerTrajectory::State getInitialState() {
		return m_states[0];
	}

	/**
	 * Get the initial target pose for a holonomic drivetrain NOTE: This is a "target" pose, meaning
	 * the rotation will be the value of the next rotation target along the path, not what the
	 * rotation should be at the start of the path
	 *
	 * @return The initial target pose
	 */
	inline frc::Pose2d getInitialTargetHolonomicPose() {
		return m_states[0].getTargetHolonomicPose();
	}

	/**
	 * Get this initial pose for a differential drivetrain
	 *
	 * @return The initial pose
	 */
	inline frc::Pose2d getInitialDifferentialPose() {
		return m_states[0].getDifferentialPose();
	}

	/**
	 * Get the end state of the trajectory
	 *
	 * @return The end state
	 */
	inline PathPlannerTrajectory::State getEndState() {
		return m_states[m_states.size() - 1];
	}

private:
	std::vector<PathPlannerTrajectory::State> m_states;

	static size_t getNextRotationTargetIdx(
			std::shared_ptr<PathPlannerPath> path, const size_t startingIndex);

	static std::vector<PathPlannerTrajectory::State> generateStates(
			std::shared_ptr<PathPlannerPath> path,
			const frc::ChassisSpeeds &startingSpeeds,
			const frc::Rotation2d &startingRotation);
};
}