/**
 * @file externalPose.h
 *
 * @brief Provides a consistent API for updating and accessing the external pose of the robot swarm.
 *
 * @details This was developed as an interface between poses provided by the Newton Cam and the
 * AprilTag system, but will work for any system that provides ASCII radio commands of the form:
 *
 * 		"EP, ID, X,Y,theta"
 *
 * Where 'X','Y','theta' are integers.
 * To use this, call externalPoseInit(); in your behaviorTask()
 * @see "\robotcode\AprilTagReader\src\AprilTagReaderUsageExample.c"
 *
 * @since November 28, 2012
 * @author James McLurkin
 */

#ifndef EXTERNALPOSE_H_
#define EXTERNALPOSE_H_

//TODO Check brief correctness
/**
 * @brief External pose of the robot as determined by an outside ASCII radio command system.
 */
typedef struct ExternalPose {
	uint8 ID;			/**< ID of the robot */
	Pose pose;			/**< Robot's pose */
	uint32 timeStamp;	/**< Timestamp of the ExternalPose */
	boolean active;		/**< Status of the ExternalPose (TRUE for active, FALSE otherwise) */
} ExternalPose;

/**
 * @brief sets up memory and radio callbacks for getting external pose data.
 *
 * Should be called once in behaviorTask.
 * @returns void
 */
void externalPoseInit(void);


/**
 * @brief Gets this robot's pose.
 *
 * @param poseArg (overwritten if external pose is active with external pose data)
 * @returns TRUE if external pose is active
 */
boolean externalPoseGet(ExternalPose* poseArg);


/**
 * @brief Gets the pose of a given neighbor.
 *
 * @param poseArg (overwritten with external pose data if external pose is active and a neighbor)
 * @param nbrPtr robot we want to know the external pose of
 * @returns TRUE if external pose is active AND this neighbor exists
 */
boolean externalPoseGetNbr(ExternalPose* poseArg, Nbr* nbrPtr);


/**
 * @brief Gets the distance to a given neighbor's external pose.
 *
 * @param nbrPtr for robot we want to know the distance from
 * @returns the distance to the given neighbor's external pose.
 */
int32 externalPoseGetNbrRange(Nbr* nbrPtr);


/**
 * @brief Determines if this is "the Host" (the robot connected over serial to external system).
 *
 * @returns TRUE if this is the host or FALSE
 */
boolean externalPoseIsHost(void);


/**
 * @brief Determines if the external pose is active or inactive.
 *
 * @returns TRUE if external pose is active or FALSE if not
 */
boolean externalPoseIsActive(void);

#endif /* EXTERNALPOSE_H_ */
