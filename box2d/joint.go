package box2d

/*
#include "box2d/box2d.h"
*/
import "C"

// Joint id references a joint instance. This should be treated as an opaque handle.
type JointId struct {
	index1     int32
	world0     uint16
	generation uint16
}

/*
// Destroy a joint. Optionally wake attached bodies.
B2_API void b2DestroyJoint( b2JointId jointId, bool wakeAttached );

// Joint identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Joint_IsValid( b2JointId id );

// Get the joint type
B2_API b2JointType b2Joint_GetType( b2JointId jointId );

// Get body A id on a joint
B2_API b2BodyId b2Joint_GetBodyA( b2JointId jointId );

// Get body B id on a joint
B2_API b2BodyId b2Joint_GetBodyB( b2JointId jointId );

// Get the world that owns this joint
B2_API b2WorldId b2Joint_GetWorld( b2JointId jointId );

// Set the local frame on bodyA
B2_API void b2Joint_SetLocalFrameA( b2JointId jointId, Transform localFrame );

// Get the local frame on bodyA
B2_API Transform b2Joint_GetLocalFrameA( b2JointId jointId );

// Set the local frame on bodyB
B2_API void b2Joint_SetLocalFrameB( b2JointId jointId, Transform localFrame );

// Get the local frame on bodyB
B2_API Transform b2Joint_GetLocalFrameB( b2JointId jointId );

// Toggle collision between connected bodies
B2_API void b2Joint_SetCollideConnected( b2JointId jointId, bool shouldCollide );

// Is collision allowed between connected bodies?
B2_API bool b2Joint_GetCollideConnected( b2JointId jointId );

// Set the user data on a joint
B2_API void b2Joint_SetUserData( b2JointId jointId, void* userData );

// Get the user data on a joint
B2_API void* b2Joint_GetUserData( b2JointId jointId );

// Wake the bodies connect to this joint
B2_API void b2Joint_WakeBodies( b2JointId jointId );

// Get the current constraint force for this joint. Usually in Newtons.
B2_API Vec2 b2Joint_GetConstraintForce( b2JointId jointId );

// Get the current constraint torque for this joint. Usually in Newton * meters.
B2_API float32 b2Joint_GetConstraintTorque( b2JointId jointId );

// Get the current linear separation error for this joint. Does not consider admissible movement. Usually in meters.
B2_API float32 b2Joint_GetLinearSeparation( b2JointId jointId );

// Get the current angular separation error for this joint. Does not consider admissible movement. Usually in meters.
B2_API float32 b2Joint_GetAngularSeparation( b2JointId jointId );

// Set the joint constraint tuning. Advanced feature.
// @param jointId the joint
// @param hertz the stiffness in Hertz (cycles per second)
// @param dampingRatio the non-dimensional damping ratio (one for critical damping)
B2_API void b2Joint_SetConstraintTuning( b2JointId jointId, float32 hertz, float32 dampingRatio );

// Get the joint constraint tuning. Advanced feature.
B2_API void b2Joint_GetConstraintTuning( b2JointId jointId, float32* hertz, float32* dampingRatio );

// Set the force threshold for joint events (Newtons)
B2_API void b2Joint_SetForceThreshold( b2JointId jointId, float32 threshold );

// Get the force threshold for joint events (Newtons)
B2_API float32 b2Joint_GetForceThreshold( b2JointId jointId );

// Set the torque threshold for joint events (N-m)
B2_API void b2Joint_SetTorqueThreshold( b2JointId jointId, float32 threshold );

// Get the torque threshold for joint events (N-m)
B2_API float32 b2Joint_GetTorqueThreshold( b2JointId jointId );

//
// @defgroup distance_joint Distance Joint
// @brief Functions for the distance joint.
//

// Create a distance joint
// @see b2DistanceJointDef for details
B2_API b2JointId b2CreateDistanceJoint( b2WorldId worldId, const b2DistanceJointDef* def );

// Set the rest length of a distance joint
// @param jointId The id for a distance joint
// @param length The new distance joint length
B2_API void b2DistanceJoint_SetLength( b2JointId jointId, float32 length );

// Get the rest length of a distance joint
B2_API float32 b2DistanceJoint_GetLength( b2JointId jointId );

// Enable/disable the distance joint spring. When disabled the distance joint is rigid.
B2_API void b2DistanceJoint_EnableSpring( b2JointId jointId, bool enableSpring );

// Is the distance joint spring enabled?
B2_API bool b2DistanceJoint_IsSpringEnabled( b2JointId jointId );

// Set the force range for the spring.
B2_API void b2DistanceJoint_SetSpringForceRange( b2JointId jointId, float32 lowerForce, float32 upperForce );

// Get the force range for the spring.
B2_API void b2DistanceJoint_GetSpringForceRange( b2JointId jointId, float32* lowerForce, float32* upperForce );

// Set the spring stiffness in Hertz
B2_API void b2DistanceJoint_SetSpringHertz( b2JointId jointId, float32 hertz );

// Set the spring damping ratio, non-dimensional
B2_API void b2DistanceJoint_SetSpringDampingRatio( b2JointId jointId, float32 dampingRatio );

// Get the spring Hertz
B2_API float32 b2DistanceJoint_GetSpringHertz( b2JointId jointId );

// Get the spring damping ratio
B2_API float32 b2DistanceJoint_GetSpringDampingRatio( b2JointId jointId );

// Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
// and the limit has no effect.
B2_API void b2DistanceJoint_EnableLimit( b2JointId jointId, bool enableLimit );

// Is the distance joint limit enabled?
B2_API bool b2DistanceJoint_IsLimitEnabled( b2JointId jointId );

// Set the minimum and maximum length parameters of a distance joint
B2_API void b2DistanceJoint_SetLengthRange( b2JointId jointId, float32 minLength, float32 maxLength );

// Get the distance joint minimum length
B2_API float32 b2DistanceJoint_GetMinLength( b2JointId jointId );

// Get the distance joint maximum length
B2_API float32 b2DistanceJoint_GetMaxLength( b2JointId jointId );

// Get the current length of a distance joint
B2_API float32 b2DistanceJoint_GetCurrentLength( b2JointId jointId );

// Enable/disable the distance joint motor
B2_API void b2DistanceJoint_EnableMotor( b2JointId jointId, bool enableMotor );

// Is the distance joint motor enabled?
B2_API bool b2DistanceJoint_IsMotorEnabled( b2JointId jointId );

// Set the distance joint motor speed, usually in meters per second
B2_API void b2DistanceJoint_SetMotorSpeed( b2JointId jointId, float32 motorSpeed );

// Get the distance joint motor speed, usually in meters per second
B2_API float32 b2DistanceJoint_GetMotorSpeed( b2JointId jointId );

// Set the distance joint maximum motor force, usually in newtons
B2_API void b2DistanceJoint_SetMaxMotorForce( b2JointId jointId, float32 force );

// Get the distance joint maximum motor force, usually in newtons
B2_API float32 b2DistanceJoint_GetMaxMotorForce( b2JointId jointId );

// Get the distance joint current motor force, usually in newtons
B2_API float32 b2DistanceJoint_GetMotorForce( b2JointId jointId );

//
// @defgroup motor_joint Motor Joint
// @brief Functions for the motor joint.
//
// The motor joint is designed to control the movement of a body while still being
// responsive to collisions. A spring controls the position and rotation. A velocity motor
// can be used to control velocity and allows for friction in top-down games. Both types
// of control can be combined. For example, you can have a spring with friction.
// Position and velocity control have force and torque limits.
//

// Create a motor joint
// @see b2MotorJointDef for details
B2_API b2JointId b2CreateMotorJoint( b2WorldId worldId, const b2MotorJointDef* def );

// Set the desired relative linear velocity in meters per second
B2_API void b2MotorJoint_SetLinearVelocity( b2JointId jointId, Vec2 velocity );

// Get the desired relative linear velocity in meters per second
B2_API Vec2 b2MotorJoint_GetLinearVelocity( b2JointId jointId );

// Set the desired relative angular velocity in radians per second
B2_API void b2MotorJoint_SetAngularVelocity( b2JointId jointId, float32 velocity );

// Get the desired relative angular velocity in radians per second
B2_API float32 b2MotorJoint_GetAngularVelocity( b2JointId jointId );

// Set the motor joint maximum force, usually in newtons
B2_API void b2MotorJoint_SetMaxVelocityForce( b2JointId jointId, float32 maxForce );

// Get the motor joint maximum force, usually in newtons
B2_API float32 b2MotorJoint_GetMaxVelocityForce( b2JointId jointId );

// Set the motor joint maximum torque, usually in newton-meters
B2_API void b2MotorJoint_SetMaxVelocityTorque( b2JointId jointId, float32 maxTorque );

// Get the motor joint maximum torque, usually in newton-meters
B2_API float32 b2MotorJoint_GetMaxVelocityTorque( b2JointId jointId );

// Set the spring linear hertz stiffness
B2_API void b2MotorJoint_SetLinearHertz( b2JointId jointId, float32 hertz );

// Get the spring linear hertz stiffness
B2_API float32 b2MotorJoint_GetLinearHertz( b2JointId jointId );

// Set the spring linear damping ratio. Use 1.0 for critical damping.
B2_API void b2MotorJoint_SetLinearDampingRatio( b2JointId jointId, float32 damping );

// Get the spring linear damping ratio.
B2_API float32 b2MotorJoint_GetLinearDampingRatio( b2JointId jointId );

// Set the spring angular hertz stiffness
B2_API void b2MotorJoint_SetAngularHertz( b2JointId jointId, float32 hertz );

// Get the spring angular hertz stiffness
B2_API float32 b2MotorJoint_GetAngularHertz( b2JointId jointId );

// Set the spring angular damping ratio. Use 1.0 for critical damping.
B2_API void b2MotorJoint_SetAngularDampingRatio( b2JointId jointId, float32 damping );

// Get the spring angular damping ratio.
B2_API float32 b2MotorJoint_GetAngularDampingRatio( b2JointId jointId );

// Set the maximum spring force in newtons.
B2_API void b2MotorJoint_SetMaxSpringForce( b2JointId jointId, float32 maxForce );

// Get the maximum spring force in newtons.
B2_API float32 b2MotorJoint_GetMaxSpringForce( b2JointId jointId );

// Set the maximum spring torque in newtons * meters
B2_API void b2MotorJoint_SetMaxSpringTorque( b2JointId jointId, float32 maxTorque );

// Get the maximum spring torque in newtons * meters
B2_API float32 b2MotorJoint_GetMaxSpringTorque( b2JointId jointId );

//
// @defgroup filter_joint Filter Joint
// @brief Functions for the filter joint.
//
// The filter joint is used to disable collision between two bodies. As a side effect of being a joint, it also
// keeps the two bodies in the same simulation island.
//

// Create a filter joint.
// @see FilterJointDef for details
B2_API b2JointId b2CreateFilterJoint( b2WorldId worldId, const FilterJointDef* def );

//
// @defgroup prismatic_joint Prismatic Joint
// @brief A prismatic joint allows for translation along a single axis with no rotation.
//
// The prismatic joint is useful for things like pistons and moving platforms, where you want a body to translate
// along an axis and have no rotation. Also called a *slider* joint.
//

// Create a prismatic (slider) joint.
// @see b2PrismaticJointDef for details
B2_API b2JointId b2CreatePrismaticJoint( b2WorldId worldId, const b2PrismaticJointDef* def );

// Enable/disable the joint spring.
B2_API void b2PrismaticJoint_EnableSpring( b2JointId jointId, bool enableSpring );

// Is the prismatic joint spring enabled or not?
B2_API bool b2PrismaticJoint_IsSpringEnabled( b2JointId jointId );

// Set the prismatic joint stiffness in Hertz.
// This should usually be less than a quarter of the simulation rate. For example, if the simulation
// runs at 60Hz then the joint stiffness should be 15Hz or less.
B2_API void b2PrismaticJoint_SetSpringHertz( b2JointId jointId, float32 hertz );

// Get the prismatic joint stiffness in Hertz
B2_API float32 b2PrismaticJoint_GetSpringHertz( b2JointId jointId );

// Set the prismatic joint damping ratio (non-dimensional)
B2_API void b2PrismaticJoint_SetSpringDampingRatio( b2JointId jointId, float32 dampingRatio );

// Get the prismatic spring damping ratio (non-dimensional)
B2_API float32 b2PrismaticJoint_GetSpringDampingRatio( b2JointId jointId );

// Set the prismatic joint spring target angle, usually in meters
B2_API void b2PrismaticJoint_SetTargetTranslation( b2JointId jointId, float32 translation );

// Get the prismatic joint spring target translation, usually in meters
B2_API float32 b2PrismaticJoint_GetTargetTranslation( b2JointId jointId );

// Enable/disable a prismatic joint limit
B2_API void b2PrismaticJoint_EnableLimit( b2JointId jointId, bool enableLimit );

// Is the prismatic joint limit enabled?
B2_API bool b2PrismaticJoint_IsLimitEnabled( b2JointId jointId );

// Get the prismatic joint lower limit
B2_API float32 b2PrismaticJoint_GetLowerLimit( b2JointId jointId );

// Get the prismatic joint upper limit
B2_API float32 b2PrismaticJoint_GetUpperLimit( b2JointId jointId );

// Set the prismatic joint limits
B2_API void b2PrismaticJoint_SetLimits( b2JointId jointId, float32 lower, float32 upper );

// Enable/disable a prismatic joint motor
B2_API void b2PrismaticJoint_EnableMotor( b2JointId jointId, bool enableMotor );

// Is the prismatic joint motor enabled?
B2_API bool b2PrismaticJoint_IsMotorEnabled( b2JointId jointId );

// Set the prismatic joint motor speed, usually in meters per second
B2_API void b2PrismaticJoint_SetMotorSpeed( b2JointId jointId, float32 motorSpeed );

// Get the prismatic joint motor speed, usually in meters per second
B2_API float32 b2PrismaticJoint_GetMotorSpeed( b2JointId jointId );

// Set the prismatic joint maximum motor force, usually in newtons
B2_API void b2PrismaticJoint_SetMaxMotorForce( b2JointId jointId, float32 force );

// Get the prismatic joint maximum motor force, usually in newtons
B2_API float32 b2PrismaticJoint_GetMaxMotorForce( b2JointId jointId );

// Get the prismatic joint current motor force, usually in newtons
B2_API float32 b2PrismaticJoint_GetMotorForce( b2JointId jointId );

// Get the current joint translation, usually in meters.
B2_API float32 b2PrismaticJoint_GetTranslation( b2JointId jointId );

// Get the current joint translation speed, usually in meters per second.
B2_API float32 b2PrismaticJoint_GetSpeed( b2JointId jointId );

//
// @defgroup revolute_joint Revolute Joint
// @brief A revolute joint allows for relative rotation in the 2D plane with no relative translation.
//
// The revolute joint is probably the most common joint. It can be used for ragdolls and chains.
// Also called a *hinge* or *pin* joint.
//

// Create a revolute joint
// @see b2RevoluteJointDef for details
B2_API b2JointId b2CreateRevoluteJoint( b2WorldId worldId, const b2RevoluteJointDef* def );

// Enable/disable the revolute joint spring
B2_API void b2RevoluteJoint_EnableSpring( b2JointId jointId, bool enableSpring );

// It the revolute angular spring enabled?
B2_API bool b2RevoluteJoint_IsSpringEnabled( b2JointId jointId );

// Set the revolute joint spring stiffness in Hertz
B2_API void b2RevoluteJoint_SetSpringHertz( b2JointId jointId, float32 hertz );

// Get the revolute joint spring stiffness in Hertz
B2_API float32 b2RevoluteJoint_GetSpringHertz( b2JointId jointId );

// Set the revolute joint spring damping ratio, non-dimensional
B2_API void b2RevoluteJoint_SetSpringDampingRatio( b2JointId jointId, float32 dampingRatio );

// Get the revolute joint spring damping ratio, non-dimensional
B2_API float32 b2RevoluteJoint_GetSpringDampingRatio( b2JointId jointId );

// Set the revolute joint spring target angle, radians
B2_API void b2RevoluteJoint_SetTargetAngle( b2JointId jointId, float32 angle );

// Get the revolute joint spring target angle, radians
B2_API float32 b2RevoluteJoint_GetTargetAngle( b2JointId jointId );

// Get the revolute joint current angle in radians relative to the reference angle
// @see b2RevoluteJointDef::referenceAngle
B2_API float32 b2RevoluteJoint_GetAngle( b2JointId jointId );

// Enable/disable the revolute joint limit
B2_API void b2RevoluteJoint_EnableLimit( b2JointId jointId, bool enableLimit );

// Is the revolute joint limit enabled?
B2_API bool b2RevoluteJoint_IsLimitEnabled( b2JointId jointId );

// Get the revolute joint lower limit in radians
B2_API float32 b2RevoluteJoint_GetLowerLimit( b2JointId jointId );

// Get the revolute joint upper limit in radians
B2_API float32 b2RevoluteJoint_GetUpperLimit( b2JointId jointId );

// Set the revolute joint limits in radians. It is expected that lower <= upper
// and that -0.99 * B2_PI <= lower && upper <= -0.99 * B2_PI.
B2_API void b2RevoluteJoint_SetLimits( b2JointId jointId, float32 lower, float32 upper );

// Enable/disable a revolute joint motor
B2_API void b2RevoluteJoint_EnableMotor( b2JointId jointId, bool enableMotor );

// Is the revolute joint motor enabled?
B2_API bool b2RevoluteJoint_IsMotorEnabled( b2JointId jointId );

// Set the revolute joint motor speed in radians per second
B2_API void b2RevoluteJoint_SetMotorSpeed( b2JointId jointId, float32 motorSpeed );

// Get the revolute joint motor speed in radians per second
B2_API float32 b2RevoluteJoint_GetMotorSpeed( b2JointId jointId );

// Get the revolute joint current motor torque, usually in newton-meters
B2_API float32 b2RevoluteJoint_GetMotorTorque( b2JointId jointId );

// Set the revolute joint maximum motor torque, usually in newton-meters
B2_API void b2RevoluteJoint_SetMaxMotorTorque( b2JointId jointId, float32 torque );

// Get the revolute joint maximum motor torque, usually in newton-meters
B2_API float32 b2RevoluteJoint_GetMaxMotorTorque( b2JointId jointId );

//
// @defgroup weld_joint Weld Joint
// @brief A weld joint fully constrains the relative transform between two bodies while allowing for springiness
//
// A weld joint constrains the relative rotation and translation between two bodies. Both rotation and translation
// can have damped springs.
//
// @note The accuracy of weld joint is limited by the accuracy of the solver. Long chains of weld joints may flex.
//

// Create a weld joint
// @see b2WeldJointDef for details
B2_API b2JointId b2CreateWeldJoint( b2WorldId worldId, const b2WeldJointDef* def );

// Set the weld joint linear stiffness in Hertz. 0 is rigid.
B2_API void b2WeldJoint_SetLinearHertz( b2JointId jointId, float32 hertz );

// Get the weld joint linear stiffness in Hertz
B2_API float32 b2WeldJoint_GetLinearHertz( b2JointId jointId );

// Set the weld joint linear damping ratio (non-dimensional)
B2_API void b2WeldJoint_SetLinearDampingRatio( b2JointId jointId, float32 dampingRatio );

// Get the weld joint linear damping ratio (non-dimensional)
B2_API float32 b2WeldJoint_GetLinearDampingRatio( b2JointId jointId );

// Set the weld joint angular stiffness in Hertz. 0 is rigid.
B2_API void b2WeldJoint_SetAngularHertz( b2JointId jointId, float32 hertz );

// Get the weld joint angular stiffness in Hertz
B2_API float32 b2WeldJoint_GetAngularHertz( b2JointId jointId );

// Set weld joint angular damping ratio, non-dimensional
B2_API void b2WeldJoint_SetAngularDampingRatio( b2JointId jointId, float32 dampingRatio );

// Get the weld joint angular damping ratio, non-dimensional
B2_API float32 b2WeldJoint_GetAngularDampingRatio( b2JointId jointId );

//
// @defgroup wheel_joint Wheel Joint
// The wheel joint can be used to simulate wheels on vehicles.
//
// The wheel joint restricts body B to move along a local axis in body A. Body B is free to
// rotate. Supports a linear spring, linear limits, and a rotational motor.
//

// Create a wheel joint
// @see b2WheelJointDef for details
B2_API b2JointId b2CreateWheelJoint( b2WorldId worldId, const b2WheelJointDef* def );

// Enable/disable the wheel joint spring
B2_API void b2WheelJoint_EnableSpring( b2JointId jointId, bool enableSpring );

// Is the wheel joint spring enabled?
B2_API bool b2WheelJoint_IsSpringEnabled( b2JointId jointId );

// Set the wheel joint stiffness in Hertz
B2_API void b2WheelJoint_SetSpringHertz( b2JointId jointId, float32 hertz );

// Get the wheel joint stiffness in Hertz
B2_API float32 b2WheelJoint_GetSpringHertz( b2JointId jointId );

// Set the wheel joint damping ratio, non-dimensional
B2_API void b2WheelJoint_SetSpringDampingRatio( b2JointId jointId, float32 dampingRatio );

// Get the wheel joint damping ratio, non-dimensional
B2_API float32 b2WheelJoint_GetSpringDampingRatio( b2JointId jointId );

// Enable/disable the wheel joint limit
B2_API void b2WheelJoint_EnableLimit( b2JointId jointId, bool enableLimit );

// Is the wheel joint limit enabled?
B2_API bool b2WheelJoint_IsLimitEnabled( b2JointId jointId );

// Get the wheel joint lower limit
B2_API float32 b2WheelJoint_GetLowerLimit( b2JointId jointId );

// Get the wheel joint upper limit
B2_API float32 b2WheelJoint_GetUpperLimit( b2JointId jointId );

// Set the wheel joint limits
B2_API void b2WheelJoint_SetLimits( b2JointId jointId, float32 lower, float32 upper );

// Enable/disable the wheel joint motor
B2_API void b2WheelJoint_EnableMotor( b2JointId jointId, bool enableMotor );

// Is the wheel joint motor enabled?
B2_API bool b2WheelJoint_IsMotorEnabled( b2JointId jointId );

// Set the wheel joint motor speed in radians per second
B2_API void b2WheelJoint_SetMotorSpeed( b2JointId jointId, float32 motorSpeed );

// Get the wheel joint motor speed in radians per second
B2_API float32 b2WheelJoint_GetMotorSpeed( b2JointId jointId );

// Set the wheel joint maximum motor torque, usually in newton-meters
B2_API void b2WheelJoint_SetMaxMotorTorque( b2JointId jointId, float32 torque );

// Get the wheel joint maximum motor torque, usually in newton-meters
B2_API float32 b2WheelJoint_GetMaxMotorTorque( b2JointId jointId );

// Get the wheel joint current motor torque, usually in newton-meters
B2_API float32 b2WheelJoint_GetMotorTorque( b2JointId jointId );
*/
