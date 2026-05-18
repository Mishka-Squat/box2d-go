package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

import (
	"github.com/Mishka-Squat/gamemath/vector2"
)

type Vec2 = vector2.Float32
type Rot = vector2.Float32
type Transform struct {
	p Vec2
	q Rot
}

// / A 2-by-2 Matrix
type Mat22 struct {
	/// columns
	cx, cy Vec2
}

// / Axis-aligned bounding box
type AABB struct {
	lowerBound Vec2
	upperBound Vec2
}

// Task interface
// This is the prototype for a Box2D task. Your task system is expected to run this callback on a worker thread,
// exactly once per enqueue, passing back the same taskContext pointer supplied to b2EnqueueTaskCallback.
// @ingroup world
type TaskCallback = func(taskContext any)

// These functions can be provided to Box2D to invoke a task system.
// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
// serially within the callback and there is no need to call b2FinishTaskCallback.
// @ingroup world
type EnqueueTaskCallback = func(task TaskCallback, taskContext any, userContext any) any

// Finishes a user task object that wraps a Box2D task.
// @ingroup world
type FinishTaskCallback = func(userTask any, userContext any)

// Optional friction mixing callback. This intentionally provides no context objects because this is called
// from a worker thread.
// @warning This function should not attempt to modify Box2D state or user application state.
// @ingroup world
type FrictionCallback = func(frictionA float32, userMaterialIdA uint64, frictionB float32, userMaterialIdB uint64) float32

// Optional restitution mixing callback. This intentionally provides no context objects because this is called
// from a worker thread.
// @warning This function should not attempt to modify Box2D state or user application state.
// @ingroup world
type RestitutionCallback = func(restitutionA float32, userMaterialIdA uint64, restitutionB float32, userMaterialIdB uint64) float32

// Result from b2World_RayCastClosest
// If there is initial overlap the fraction and normal will be zero while the point is an arbitrary point in the overlap region.
// @ingroup world
type RayResult struct {
	ShapeId    ShapeId
	Point      Vec2
	Normal     Vec2
	Fraction   float32
	NodeVisits int
	LeafVisits int
	Hit        bool
}

// Optional world capacities that can be used to avoid run-time allocations.
// @see b2World_GetMaxCapacity
// @ingroup world
type Capacity struct {
	// Number of expected static shapes.
	StaticShapeCount int

	// Number of expected dynamic and kinematic shapes.
	DynamicShapeCount int

	// Number of expected static bodies.
	StaticBodyCount int

	// Number of expected dynamic and kinematic bodies.
	DynamicBodyCount int

	// Number of expected contacts.
	ContactCount int
}

// World definition used to create a simulation world.
// Must be initialized using b2DefaultWorldDef().
// @ingroup world
type WorldDef struct {
	// Gravity vector. Box2D has no up-vector defined.
	Gravity Vec2

	// Restitution speed threshold, usually in m/s. Collisions above this
	// speed have restitution applied (will bounce).
	RestitutionThreshold float32

	// Threshold speed for hit events. Usually meters per second.
	HitEventThreshold float32

	// Contact stiffness. Cycles per second. Increasing this increases the speed of overlap recovery, but can introduce jitter.
	ContactHertz float32

	// Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this with
	// the trade-off that overlap resolution becomes more energetic.
	ContactDampingRatio float32

	// This parameter controls how fast overlap is resolved and usually has units of meters per second. This only
	// puts a cap on the resolution speed. The resolution speed is increased by increasing the hertz and/or
	// decreasing the damping ratio.
	ContactSpeed float32

	// Maximum linear speed. Usually meters per second.
	MaximumLinearSpeed float32

	// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
	FrictionCallback FrictionCallback

	// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
	RestitutionCallback RestitutionCallback

	// Can bodies go to sleep to improve performance
	EnableSleep bool

	// Enable continuous collision
	EnableContinuous bool

	// Contact softening when mass ratios are large. Experimental.
	EnableContactSoftening bool

	// Number of workers for multithreading. Box2D performs best when using performance cores and
	// accessing a single L3 cache (uniform memory). Efficiency cores and SMT provide
	// little benefit and may even harm performance.
	// This is clamped to the range [1, B2_MAX_WORKERS].
	// Using a value above 1 will turn on multithreading. If task callbacks are provided
	// then Box2D will use the user provided task system. Otherwise Box2D will create threads and use
	// an internal scheduler.
	WorkerCount int

	// Function to spawn tasks
	EnqueueTask EnqueueTaskCallback

	// Function to finish a task
	FinishTask FinishTaskCallback

	// User context that is provided to enqueueTask and finishTask
	UserTaskContext any

	// User data
	UserData any

	// Optional initial capacities
	Capacity Capacity

	// Used internally to detect a valid definition. DO NOT SET.
	InternalValue int
}

func DefaultWorldDef() WorldDef {
	r := C.b2DefaultWorldDef()
	return *cast[WorldDef](&r)
}

// The body simulation type.
// Each body is one of these three types. The type determines how the body behaves in the simulation.
// @ingroup body
type BodyType int

const (
	// zero mass, zero velocity, may be manually moved
	StaticBody BodyType = iota

	// zero mass, velocity set by user, moved by solver
	KinematicBody

	// positive mass, velocity determined by forces, moved by solver
	DynamicBody

	// number of body types
	BodyTypeCount
)

// Motion locks to restrict the body movement
type MotionLocks struct {
	// Prevent translation along the x-axis
	LinearX bool

	// Prevent translation along the y-axis
	LinearY bool

	// Prevent rotation around the z-axis
	AngularZ bool
}

// A body definition holds all the data needed to construct a rigid body.
// You can safely re-use body definitions. Shapes are added to a body after construction.
// Body definitions are temporary objects used to bundle creation parameters.
// Must be initialized using b2DefaultBodyDef().
// @ingroup body
type BodyDef struct {
	// The body type: static, kinematic, or dynamic.
	Type BodyType

	// The initial world position of the body. Bodies should be created with the desired position.
	// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	// if the body is moved after shapes have been added.
	Position Vec2

	// The initial world rotation of the body. Use b2MakeRot() if you have an angle.
	Rotation Rot

	// The initial linear velocity of the body's origin. Usually in meters per second.
	LinearVelocity Vec2

	// The initial angular velocity of the body. Radians per second.
	AngularVelocity float32

	// Linear damping is used to reduce the linear velocity. The damping parameter
	// can be larger than 1 but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	// Generally linear damping is undesirable because it makes objects move slowly
	// as if they are floating.
	LinearDamping float32

	// Angular damping is used to reduce the angular velocity. The damping parameter
	// can be larger than 1.0f but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	// Angular damping can be use slow down rotating bodies.
	AngularDamping float32

	// Scale the gravity applied to this body. Non-dimensional.
	GravityScale float32

	// Sleep speed threshold, default is 0.05 meters per second
	SleepThreshold float32

	// Optional body name for debugging. Up to 31 characters (excluding null termination)
	Name *byte

	// Use this to store application specific body data.
	UserData any

	// Motions locks to restrict linear and angular movement.
	// Caution: may lead to softer constraints along the locked direction
	MotionLocks MotionLocks

	// Set this flag to false if this body should never fall asleep.
	EnableSleep bool

	// Is this body initially awake or sleeping?
	IsAwake bool

	// Treat this body as a high speed object that performs continuous collision detection
	// against dynamic and kinematic bodies, but not other bullet bodies.
	// @warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	// continuous collision. They do not guarantee accurate collision if both bodies are fast moving because
	// the bullet does a continuous check after all non-bullet bodies have moved. You could get unlucky and have
	// the bullet body end a time step very close to a non-bullet body and the non-bullet body then moves over
	// the bullet body. In continuous collision, initial overlap is ignored to avoid freezing bodies in place.
	// I do not recommend using them for game projectiles if precise collision timing is needed. Instead consider
	// using a ray or shape cast. You can use a marching ray or shape cast for projectile that moves over time.
	// If you want a fast moving projectile to collide with a fast moving target, you need to consider the relative
	// movement in your ray or shape cast. This is out of the scope of Box2D.
	// So what are good use cases for bullets? Pinball games or games with dynamic containers that hold other objects.
	// It should be a use case where it doesn't break the game if there is a collision missed, but having them
	// captured improves the quality of the game.
	IsBullet bool

	// Used to disable a body. A disabled body does not move or collide.
	IsEnabled bool

	// This allows this body to bypass rotational speed limits. Should only be used
	// for circular objects, like wheels.
	AllowFastRotation bool

	// Used internally to detect a valid definition. DO NOT SET.
	InternalValue int
}

func DefaultBodyDef() BodyDef {
	r := C.b2DefaultBodyDef()
	return *cast[BodyDef](&r)
}

// This is used to filter collision on shapes. It affects shape-vs-shape collision
// and shape-versus-query collision (such as b2World_CastRay).
// @ingroup shape
type Filter struct {
	// The collision category bits. Normally you would just set one bit. The category bits should
	// represent your application object types. For example:
	// @code{.cpp}
	// enum MyCategories
	// {
	//    Static  = 0x00000001,
	//    Dynamic = 0x00000002,
	//    Debris  = 0x00000004,
	//    Player  = 0x00000008,
	//    // etc
	// };
	// @endcode
	CategoryBits uint64

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	// For example, you may want your player to only collide with static objects
	// and other players.
	// @code{.c}
	// maskBits = Static | Player;
	// @endcode
	MaskBits uint64

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	// always wins against the mask bits.
	// For example, you may want ragdolls to collide with other ragdolls but you don't want
	// ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	// and apply that group index to all shapes on the ragdoll.
	GroupIndex int
}

// Use this to initialize your filter
// @ingroup shape
func DefaultFilter() Filter {
	r := C.b2DefaultFilter()
	return *cast[Filter](&r)
}

// The query filter is used to filter collisions between queries and shapes. For example,
// you may want a ray-cast representing a projectile to hit players and the static environment
// but not debris.
// @ingroup shape
type QueryFilter struct {
	// The collision category bits of this query. Normally you would just set one bit.
	CategoryBits uint64

	// The collision mask bits. This states the shape categories that this
	// query would accept for collision.
	MaskBits uint64
}

// Use this to initialize your query filter
// @ingroup shape
func DefaultQueryFilter() QueryFilter {
	r := C.b2DefaultQueryFilter()
	return *cast[QueryFilter](&r)
}

/*

// Shape type
// @ingroup shape
typedef enum b2ShapeType
{
	// A circle with an offset
	b2_circleShape,

	// A capsule is an extruded circle
	b2_capsuleShape,

	// A line segment
	b2_segmentShape,

	// A convex polygon
	b2_polygonShape,

	// A line segment owned by a chain shape
	b2_chainSegmentShape,

	// The number of shape types
	b2_shapeTypeCount
} b2ShapeType;

// Surface materials allow chain shapes to have per segment surface properties.
// @ingroup shape
type  b2SurfaceMaterial struct {
	// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	float friction;

	// The coefficient of restitution (bounce) usually in the range [0,1].
	// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	float restitution;

	// The rolling resistance usually in the range [0,1].
	float rollingResistance;

	// The tangent speed for conveyor belts
	float tangentSpeed;

	// User material identifier. This is passed with query results and to friction and restitution
	// combining functions. It is not used internally.
	uint64_t userMaterialId;

	// Custom debug draw color.
	uint32_t customColor;
}

// Use this to initialize your surface material
// @ingroup shape
B2_API b2SurfaceMaterial b2DefaultSurfaceMaterial( void );

// Used to create a shape.
// This is a temporary object used to bundle shape creation parameters. You may use
// the same shape definition to create multiple shapes.
// Must be initialized using b2DefaultShapeDef().
// @ingroup shape
type  b2ShapeDef struct {
	// Use this to store application specific shape data.
	any userData;

	// The surface material for this shape.
	b2SurfaceMaterial material;

	// The density, usually in kg/m^2.
	// This is not part of the surface material because this is for the interior, which may have
	// other considerations, such as being hollow. For example a wood barrel may be hollow or full of water.
	float density;

	// Collision filtering data.
	b2Filter filter;

	// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See b2WorldDef.
	bool enableCustomFiltering;

	// A sensor shape generates overlap events but never generates a collision response.
	// Sensors do not have continuous collision. Instead, use a ray or shape cast for those scenarios.
	// Sensors still contribute to the body mass if they have non-zero density.
	// @note Sensor events are disabled by default.
	// @see enableSensorEvents
	bool isSensor;

	// Enable sensor events for this shape. This applies to sensors and non-sensors. Both shapes involved must have this flag set to true.
	// False by default, even for sensors.
	bool enableSensorEvents;

	// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Only one shape involved needs this flag set to true.
	// Ignored for sensors. False by default.
	bool enableContactEvents;

	// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Only one shape involved needs this flag set to true.
	// Ignored for sensors. False by default.
	bool enableHitEvents;

	// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	// and must be carefully handled due to multithreading. Ignored for sensors.
	bool enablePreSolveEvents;

	// When shapes are created they will scan the environment for collision the next time step. This can significantly slow down
	// static body creation when there are many static shapes.
	// This is flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
	bool invokeContactCreation;

	// Should the body update the mass properties when this shape is created. Default is true.
	// Warning: if this is true, you MUST call b2Body_ApplyMassFromShapes before simulating the world.
	bool updateBodyMass;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

// Use this to initialize your shape definition
// @ingroup shape
B2_API b2ShapeDef b2DefaultShapeDef( void );

// Used to create a chain of line segments. This is designed to eliminate ghost collisions with some limitations.
// - chains are one-sided
// - chains have no mass and should be used on static bodies
// - chains have a counter-clockwise winding order (normal points right of segment direction)
// - chains are either a loop or open
// - a chain must have at least 4 points
// - the distance between any two points must be greater than B2_LINEAR_SLOP
// - a chain shape should not self intersect (this is not validated)
// - an open chain shape has NO COLLISION on the first and final edge
// - you may overlap two open chains on their first three and/or last three points to get smooth collision
// - a chain shape creates multiple line segment shapes on the body
// https://en.wikipedia.org/wiki/Polygonal_chain
// Must be initialized using b2DefaultChainDef().
// @warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
// @ingroup shape
type  b2ChainDef struct {
	// Use this to store application specific shape data.
	any userData;

	// An array of at least 4 points. These are cloned and may be temporary.
	const b2Vec2* points;

	// The point count, must be 4 or more.
	int count;

	// Surface materials for each segment. These are cloned.
	const b2SurfaceMaterial* materials;

	// The material count. Must be 1 or count. This allows you to provide one
	// material for all segments or a unique material per segment. For open
	// chains, the material on the ghost segments are place holders.
	int materialCount;

	// Contact filtering data.
	b2Filter filter;

	// Indicates a closed chain formed by connecting the first and last points
	bool isLoop;

	// Enable sensors to detect this chain. False by default.
	bool enableSensorEvents;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

// Use this to initialize your chain definition
// @ingroup shape
B2_API b2ChainDef b2DefaultChainDef( void );

//! @cond
// Profiling data. Times are in milliseconds.
type  b2Profile struct {
	float step;
	float pairs;
	float collide;
	float solve;
	float solverSetup;
	float constraints;
	float prepareConstraints;
	float integrateVelocities;
	float warmStart;
	float solveImpulses;
	float integratePositions;
	float relaxImpulses;
	float applyRestitution;
	float storeImpulses;
	float splitIslands;
	float transforms;
	float sensorHits;
	float jointEvents;
	float hitEvents;
	float refit;
	float bullets;
	float sleepIslands;
	float sensors;
} b2Profile;

// Counters that give details of the simulation size.
type  b2Counters struct {
	int bodyCount;
	int shapeCount;
	int contactCount;
	int jointCount;
	int islandCount;
	int stackUsed;
	int staticTreeHeight;
	int treeHeight;
	int byteCount;
	int taskCount;
	int colorCounts[24];

	// Number of contacts touched by the collide pass (graph contacts + awake-set non-touching).
	int awakeContactCount;

	// Number of contacts recycled in the most recent step.
	int recycledContactCount;
} b2Counters;
//! @endcond

// Joint type enumeration
//
// This is useful because all joint types use b2JointId and sometimes you
// want to get the type of a joint.
// @ingroup joint
typedef enum b2JointType
{
	b2_distanceJoint,
	b2_filterJoint,
	b2_motorJoint,
	b2_prismaticJoint,
	b2_revoluteJoint,
	b2_weldJoint,
	b2_wheelJoint,
} b2JointType;

// Base joint definition used by all joint types.
// The local frames are measured from the body's origin rather than the center of mass because:
// 1. you might not know where the center of mass will be
// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
type  b2JointDef struct {
	// User data pointer
	any userData;

	// The first attached body
	BodyId bodyIdA;

	// The second attached body
	BodyId bodyIdB;

	// The first local joint frame
	Transform localFrameA;

	// The second local joint frame
	Transform localFrameB;

	// Force threshold for joint events
	float forceThreshold;

	// Torque threshold for joint events
	float torqueThreshold;

	// Constraint hertz (advanced feature)
	float constraintHertz;

	// Constraint damping ratio (advanced feature)
	float constraintDampingRatio;

	// Debug draw scale
	float drawScale;

	// Set this flag to true if the attached bodies should collide
	bool collideConnected;

} b2JointDef;

// Distance joint definition
// Connects a point on body A with a point on body B by a segment.
// Useful for ropes and springs.
// @ingroup distance_joint
type  b2DistanceJointDef struct {
	// Base joint definition
	b2JointDef base;

	// The rest length of this joint. Clamped to a stable minimum value.
	float length;

	// Enable the distance constraint to behave like a spring. If false
	// then the distance joint will be rigid, overriding the limit and motor.
	bool enableSpring;

	// The lower spring force controls how much tension it can sustain
	float lowerSpringForce;

	// The upper spring force controls how much compression it an sustain
	float upperSpringForce;

	// The spring linear stiffness Hertz, cycles per second
	float hertz;

	// The spring linear damping ratio, non-dimensional
	float dampingRatio;

	// Enable/disable the joint limit
	bool enableLimit;

	// Minimum length for limit. Clamped to a stable minimum value.
	float minLength;

	// Maximum length for limit. Must be greater than or equal to the minimum length.
	float maxLength;

	// Enable/disable the joint motor
	bool enableMotor;

	// The maximum motor force, usually in newtons
	float maxMotorForce;

	// The desired motor speed, usually in meters per second
	float motorSpeed;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b2DistanceJointDef;

// Use this to initialize your joint definition
// @ingroup distance_joint
B2_API b2DistanceJointDef b2DefaultDistanceJointDef( void );

// A motor joint is used to control the relative velocity and or transform between two bodies.
// With a velocity of zero this acts like top-down friction.
// @ingroup motor_joint
type  b2MotorJointDef struct {
	// Base joint definition
	b2JointDef base;

	// The desired linear velocity
	b2Vec2 linearVelocity;

	// The maximum motor force in newtons
	float maxVelocityForce;

	// The desired angular velocity
	float angularVelocity;

	// The maximum motor torque in newton-meters
	float maxVelocityTorque;

	// Linear spring hertz for position control
	float linearHertz;

	// Linear spring damping ratio
	float linearDampingRatio;

	// Maximum spring force in newtons
	float maxSpringForce;

	// Angular spring hertz for position control
	float angularHertz;

	// Angular spring damping ratio
	float angularDampingRatio;

	// Maximum spring torque in newton-meters
	float maxSpringTorque;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b2MotorJointDef;

// Use this to initialize your joint definition
// @ingroup motor_joint
B2_API b2MotorJointDef b2DefaultMotorJointDef( void );

// A filter joint is used to disable collision between two specific bodies.
//
// @ingroup filter_joint
type  b2FilterJointDef struct {
	// Base joint definition
	b2JointDef base;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b2FilterJointDef;

// Use this to initialize your joint definition
// @ingroup filter_joint
B2_API b2FilterJointDef b2DefaultFilterJointDef( void );

// Prismatic joint definition
// Body B may slide along the x-axis in local frame A. Body B cannot rotate relative to body A.
// The joint translation is zero when the local frame origins coincide in world space.
// @ingroup prismatic_joint
type  b2PrismaticJointDef struct {
	// Base joint definition
	b2JointDef base;

	// Enable a linear spring along the prismatic joint axis
	bool enableSpring;

	// The spring stiffness Hertz, cycles per second
	float hertz;

	// The spring damping ratio, non-dimensional
	float dampingRatio;

	// The target translation for the joint in meters. The spring-damper will drive
	// to this translation.
	float targetTranslation;

	// Enable/disable the joint limit
	bool enableLimit;

	// The lower translation limit
	float lowerTranslation;

	// The upper translation limit
	float upperTranslation;

	// Enable/disable the joint motor
	bool enableMotor;

	// The maximum motor force, typically in newtons
	float maxMotorForce;

	// The desired motor speed, typically in meters per second
	float motorSpeed;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b2PrismaticJointDef;

// Use this to initialize your joint definition
// @ingroup prismatic_joint
B2_API b2PrismaticJointDef b2DefaultPrismaticJointDef( void );

// Revolute joint definition
// A point on body B is fixed to a point on body A. Allows relative rotation.
// @ingroup revolute_joint
type  b2RevoluteJointDef struct {
	// Base joint definition
	b2JointDef base;

	// The target angle for the joint in radians. The spring-damper will drive
	// to this angle.
	float targetAngle;

	// Enable a rotational spring on the revolute hinge axis
	bool enableSpring;

	// The spring stiffness Hertz, cycles per second
	float hertz;

	// The spring damping ratio, non-dimensional
	float dampingRatio;

	// A flag to enable joint limits
	bool enableLimit;

	// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
	float lowerAngle;

	// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
	float upperAngle;

	// A flag to enable the joint motor
	bool enableMotor;

	// The maximum motor torque, typically in newton-meters
	float maxMotorTorque;

	// The desired motor speed in radians per second
	float motorSpeed;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b2RevoluteJointDef;

// Use this to initialize your joint definition.
// @ingroup revolute_joint
B2_API b2RevoluteJointDef b2DefaultRevoluteJointDef( void );

// Weld joint definition
// Connects two bodies together rigidly. This constraint provides springs to mimic
// soft-body simulation.
// @note The approximate solver in Box2D cannot hold many bodies together rigidly
// @ingroup weld_joint
type  b2WeldJointDef struct {
	// Base joint definition
	b2JointDef base;

	// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	float linearHertz;

	// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	float angularHertz;

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	float linearDampingRatio;

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	float angularDampingRatio;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b2WeldJointDef;

// Use this to initialize your joint definition
// @ingroup weld_joint
B2_API b2WeldJointDef b2DefaultWeldJointDef( void );

// Wheel joint definition
// Body B is a wheel that may rotate freely and slide along the local x-axis in frame A.
// The joint translation is zero when the local frame origins coincide in world space.
// @ingroup wheel_joint
type  b2WheelJointDef struct {
	// Base joint definition
	b2JointDef base;

	// Enable a linear spring along the local axis
	bool enableSpring;

	// Spring stiffness in Hertz
	float hertz;

	// Spring damping ratio, non-dimensional
	float dampingRatio;

	// Enable/disable the joint linear limit
	bool enableLimit;

	// The lower translation limit
	float lowerTranslation;

	// The upper translation limit
	float upperTranslation;

	// Enable/disable the joint rotational motor
	bool enableMotor;

	// The maximum motor torque, typically in newton-meters
	float maxMotorTorque;

	// The desired motor speed in radians per second
	float motorSpeed;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b2WheelJointDef;

// Use this to initialize your joint definition
// @ingroup wheel_joint
B2_API b2WheelJointDef b2DefaultWheelJointDef( void );

// The explosion definition is used to configure options for explosions. Explosions
// consider shape geometry when computing the impulse.
// @ingroup world
type  b2ExplosionDef struct {
	// Mask bits to filter shapes
	uint64_t maskBits;

	// The center of the explosion in world space
	b2Vec2 position;

	// The radius of the explosion
	float radius;

	// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	float falloff;

	// Impulse per unit length. This applies an impulse according to the shape perimeter that
	// is facing the explosion. Explosions only apply to circles, capsules, and polygons. This
	// may be negative for implosions.
	float impulsePerLength;
} b2ExplosionDef;

// Use this to initialize your explosion definition
// @ingroup world
B2_API b2ExplosionDef b2DefaultExplosionDef( void );

//
// @defgroup events Events
// World event types.
//
// Events are used to collect events that occur during the world time step. These events
// are then available to query after the time step is complete. This is preferable to callbacks
// because Box2D uses multithreaded simulation.
//
// Also when events occur in the simulation step it may be problematic to modify the world, which is
// often what applications want to do when events occur.
//
// With event arrays, you can scan the events in a loop and modify the world. However, you need to be careful
// that some event data may become invalid. There are several samples that show how to do this safely.
//
// @{
//
*/
// A begin touch event is generated when a shape starts to overlap a sensor shape.
type b2SensorBeginTouchEvent struct {
	// The id of the sensor shape
	SensorShapeId ShapeId

	// The id of the shape that began touching the sensor shape
	VisitorShapeId ShapeId
}

// An end touch event is generated when a shape stops overlapping a sensor shape.
//
//	These include things like setting the transform, destroying a body or shape, or changing
//	a filter. You will also get an end event if the sensor or visitor are destroyed.
//	Therefore you should always confirm the shape id is valid using b2Shape_IsValid.
type b2SensorEndTouchEvent struct {
	// The id of the sensor shape
	//	@warning this shape may have been destroyed
	//	@see b2Shape_IsValid
	SensorShapeId ShapeId

	// The id of the shape that stopped touching the sensor shape
	//	@warning this shape may have been destroyed
	//	@see b2Shape_IsValid
	VisitorShapeId ShapeId
}

// Sensor events are buffered in the world and are available
// as begin/end overlap event arrays after the time step is complete.
// Note: these may become invalid if bodies and/or shapes are destroyed
type SensorEvents struct {
	// Array of sensor begin touch events
	BeginEvents *b2SensorBeginTouchEvent

	// Array of sensor end touch events
	EndEvents *b2SensorEndTouchEvent

	// The number of begin touch events
	BeginCount int

	// The number of end touch events
	EndCount int
}

// A begin touch event is generated when two shapes begin touching.
type b2ContactBeginTouchEvent struct {
	// Id of the first shape
	ShapeIdA ShapeId

	// Id of the second shape
	ShapeIdB ShapeId

	// The transient contact id. This contact maybe destroyed automatically when the world is modified or simulated.
	// Used b2Contact_IsValid before using this id.
	ContactId ContactId
}

// An end touch event is generated when two shapes stop touching.
//
//	You will get an end event if you do anything that destroys contacts previous to the last
//	world step. These include things like setting the transform, destroying a body
//	or shape, or changing a filter or body type.
type b2ContactEndTouchEvent struct {
	// Id of the first shape
	//	@warning this shape may have been destroyed
	//	@see b2Shape_IsValid
	ShapeIdA ShapeId

	// Id of the second shape
	//	@warning this shape may have been destroyed
	//	@see b2Shape_IsValid
	ShapeIdB ShapeId

	// Id of the contact.
	//	@warning this contact may have been destroyed
	//	@see b2Contact_IsValid
	ContactId ContactId
}

// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
// This may be reported for speculative contacts that have a confirmed impulse.
type b2ContactHitEvent struct {
	// Id of the first shape
	ShapeIdA ShapeId

	// Id of the second shape
	ShapeIdB ShapeId

	// Id of the contact.
	//	@warning this contact may have been destroyed
	//	@see b2Contact_IsValid
	ContactId ContactId

	// Point where the shapes hit at the beginning of the time step.
	// This is a mid-point between the two surfaces. It could be at speculative
	// point where the two shapes were not touching at the beginning of the time step.
	Point Vec2

	// Normal vector pointing from shape A to shape B
	Normal Vec2

	// The speed the shapes are approaching. Always positive. Typically in meters per second.
	ApproachSpeed float32
}

// Contact events are buffered in the Box2D world and are available
// as event arrays after the time step is complete.
// Note: these may become invalid if bodies and/or shapes are destroyed
type ContactEvents struct {
	// Array of begin touch events
	BeginEvents *b2ContactBeginTouchEvent

	// Array of end touch events
	EndEvents *b2ContactEndTouchEvent

	// Array of hit events
	HitEvents *b2ContactHitEvent

	// Number of begin touch events
	BeginCount int

	// Number of end touch events
	EndCount int

	// Number of hit events
	HitCount int
}

// Body move events triggered when a body moves.
// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
// This also has a flag to indicate that the body went to sleep so the application can also
// sleep that actor/entity/object associated with the body.
// On the other hand if the flag does not indicate the body went to sleep then the application
// can treat the actor/entity/object associated with the body as awake.
// This is an efficient way for an application to update game object transforms rather than
// calling functions such as b2Body_GetTransform() because this data is delivered as a contiguous array
// and it is only populated with bodies that have moved.
// @note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
type BodyMoveEvent struct {
	UserData   any
	Transform  Transform
	BodyId     BodyId
	FellAsleep bool
}

// Body events are buffered in the Box2D world and are available
// as event arrays after the time step is complete.
// Note: this data becomes invalid if bodies are destroyed
type BodyEvents struct {
	// Array of move events
	MoveEvents *BodyMoveEvent

	// Number of move events
	MoveCount int
}

// Joint events report joints that are awake and have a force and/or torque exceeding the threshold
// The observed forces and torques are not returned for efficiency reasons.
type JointEvent struct {
	// The joint id
	JointId JointId

	// The user data from the joint for convenience
	UserData any
}

// Joint events are buffered in the world and are available
// as event arrays after the time step is complete.
// Note: this data becomes invalid if joints are destroyed
type JointEvents struct {
	// Array of events
	JointEvents *JointEvent

	// Number of events
	Count int
}

/*

// The contact data for two shapes. By convention the manifold normal points
// from shape A to shape B.
// @see b2Shape_GetContactData() and b2Body_GetContactData()
type  b2ContactData struct {
	b2ContactId contactId;
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
	b2Manifold manifold;
} b2ContactData;

// Prototype for a contact filter callback.
// This is called when a contact pair is considered for collision. This allows you to
// perform custom logic to prevent collision between shapes. This is only called if
// one of the two shapes has custom filtering enabled.
// Notes:
// - this function must be thread-safe
// - this is only called if one of the two shapes has enabled custom filtering
// - this may be called for awake dynamic bodies and sensors
// Return false if you want to disable the collision
// @see b2ShapeDef
// @warning Do not attempt to modify the world inside this callback
// @ingroup world
typedef bool b2CustomFilterFcn( b2ShapeId shapeIdA, b2ShapeId shapeIdB, any context );

// Prototype for a pre-solve callback.
// This is called after a contact is updated. This allows you to inspect a
// contact before it goes to the solver. If you are careful, you can modify the
// contact manifold (e.g. modify the normal).
// Notes:
// - this function must be thread-safe
// - this is only called if the shape has enabled pre-solve events
// - this is called only for awake dynamic bodies
// - this is not called for sensors
// - the supplied manifold has impulse values from the previous step
// Return false if you want to disable the contact this step
// @warning Do not attempt to modify the world inside this callback
// @ingroup world
typedef bool b2PreSolveFcn( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Vec2 point, b2Vec2 normal, any context );
*/
// Prototype callback for overlap queries.
// Called for each shape found in the query.
// @see b2World_OverlapABB
// @return false to terminate the query.
// @ingroup world
type b2OverlapResultFcn = func(shapeId ShapeId, context any) bool

/*
// Prototype callback for ray and shape casts.
// Called for each shape found in the query. You control how the ray cast
// proceeds by returning a float:
// return -1: ignore this shape and continue
// return 0: terminate the ray cast
// return fraction: clip the ray to this point
// return 1: don't clip the ray and continue
// A cast with initial overlap will return a zero fraction and a zero normal.
// @param shapeId the shape hit by the ray
// @param point the point of initial intersection
// @param normal the normal vector at the point of intersection, zero for a shape cast with initial overlap
// @param fraction the fraction along the ray at the point of intersection, zero for a shape cast with initial overlap
// @param context the user context
// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
// @see b2World_CastRay
// @ingroup world
typedef float b2CastResultFcn( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, any context );

// Used to collect collision planes for character movers.
// Return true to continue gathering planes.
typedef bool b2PlaneResultFcn( b2ShapeId shapeId, const b2PlaneResult* plane, any context );
*/
// These colors are used for debug draw and mostly match the named SVG colors.
// See https://www.rapidtables.com/web/color/index.html
// https://johndecember.com/html/spec/colorsvg.html
// https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg
type HexColor = int

const (
	b2_colorAliceBlue            HexColor = 0xF0F8FF
	b2_colorAntiqueWhite         HexColor = 0xFAEBD7
	b2_colorAqua                 HexColor = 0x00FFFF
	b2_colorAquamarine           HexColor = 0x7FFFD4
	b2_colorAzure                HexColor = 0xF0FFFF
	b2_colorBeige                HexColor = 0xF5F5DC
	b2_colorBisque               HexColor = 0xFFE4C4
	b2_colorBlack                HexColor = 0x000000
	b2_colorBlanchedAlmond       HexColor = 0xFFEBCD
	b2_colorBlue                 HexColor = 0x0000FF
	b2_colorBlueViolet           HexColor = 0x8A2BE2
	b2_colorBrown                HexColor = 0xA52A2A
	b2_colorBurlywood            HexColor = 0xDEB887
	b2_colorCadetBlue            HexColor = 0x5F9EA0
	b2_colorChartreuse           HexColor = 0x7FFF00
	b2_colorChocolate            HexColor = 0xD2691E
	b2_colorCoral                HexColor = 0xFF7F50
	b2_colorCornflowerBlue       HexColor = 0x6495ED
	b2_colorCornsilk             HexColor = 0xFFF8DC
	b2_colorCrimson              HexColor = 0xDC143C
	b2_colorCyan                 HexColor = 0x00FFFF
	b2_colorDarkBlue             HexColor = 0x00008B
	b2_colorDarkCyan             HexColor = 0x008B8B
	b2_colorDarkGoldenRod        HexColor = 0xB8860B
	b2_colorDarkGray             HexColor = 0xA9A9A9
	b2_colorDarkGreen            HexColor = 0x006400
	b2_colorDarkKhaki            HexColor = 0xBDB76B
	b2_colorDarkMagenta          HexColor = 0x8B008B
	b2_colorDarkOliveGreen       HexColor = 0x556B2F
	b2_colorDarkOrange           HexColor = 0xFF8C00
	b2_colorDarkOrchid           HexColor = 0x9932CC
	b2_colorDarkRed              HexColor = 0x8B0000
	b2_colorDarkSalmon           HexColor = 0xE9967A
	b2_colorDarkSeaGreen         HexColor = 0x8FBC8F
	b2_colorDarkSlateBlue        HexColor = 0x483D8B
	b2_colorDarkSlateGray        HexColor = 0x2F4F4F
	b2_colorDarkTurquoise        HexColor = 0x00CED1
	b2_colorDarkViolet           HexColor = 0x9400D3
	b2_colorDeepPink             HexColor = 0xFF1493
	b2_colorDeepSkyBlue          HexColor = 0x00BFFF
	b2_colorDimGray              HexColor = 0x696969
	b2_colorDodgerBlue           HexColor = 0x1E90FF
	b2_colorFireBrick            HexColor = 0xB22222
	b2_colorFloralWhite          HexColor = 0xFFFAF0
	b2_colorForestGreen          HexColor = 0x228B22
	b2_colorFuchsia              HexColor = 0xFF00FF
	b2_colorGainsboro            HexColor = 0xDCDCDC
	b2_colorGhostWhite           HexColor = 0xF8F8FF
	b2_colorGold                 HexColor = 0xFFD700
	b2_colorGoldenRod            HexColor = 0xDAA520
	b2_colorGray                 HexColor = 0x808080
	b2_colorGreen                HexColor = 0x008000
	b2_colorGreenYellow          HexColor = 0xADFF2F
	b2_colorHoneyDew             HexColor = 0xF0FFF0
	b2_colorHotPink              HexColor = 0xFF69B4
	b2_colorIndianRed            HexColor = 0xCD5C5C
	b2_colorIndigo               HexColor = 0x4B0082
	b2_colorIvory                HexColor = 0xFFFFF0
	b2_colorKhaki                HexColor = 0xF0E68C
	b2_colorLavender             HexColor = 0xE6E6FA
	b2_colorLavenderBlush        HexColor = 0xFFF0F5
	b2_colorLawnGreen            HexColor = 0x7CFC00
	b2_colorLemonChiffon         HexColor = 0xFFFACD
	b2_colorLightBlue            HexColor = 0xADD8E6
	b2_colorLightCoral           HexColor = 0xF08080
	b2_colorLightCyan            HexColor = 0xE0FFFF
	b2_colorLightGoldenRodYellow HexColor = 0xFAFAD2
	b2_colorLightGray            HexColor = 0xD3D3D3
	b2_colorLightGreen           HexColor = 0x90EE90
	b2_colorLightPink            HexColor = 0xFFB6C1
	b2_colorLightSalmon          HexColor = 0xFFA07A
	b2_colorLightSeaGreen        HexColor = 0x20B2AA
	b2_colorLightSkyBlue         HexColor = 0x87CEFA
	b2_colorLightSlateGray       HexColor = 0x778899
	b2_colorLightSteelBlue       HexColor = 0xB0C4DE
	b2_colorLightYellow          HexColor = 0xFFFFE0
	b2_colorLime                 HexColor = 0x00FF00
	b2_colorLimeGreen            HexColor = 0x32CD32
	b2_colorLinen                HexColor = 0xFAF0E6
	b2_colorMagenta              HexColor = 0xFF00FF
	b2_colorMaroon               HexColor = 0x800000
	b2_colorMediumAquaMarine     HexColor = 0x66CDAA
	b2_colorMediumBlue           HexColor = 0x0000CD
	b2_colorMediumOrchid         HexColor = 0xBA55D3
	b2_colorMediumPurple         HexColor = 0x9370DB
	b2_colorMediumSeaGreen       HexColor = 0x3CB371
	b2_colorMediumSlateBlue      HexColor = 0x7B68EE
	b2_colorMediumSpringGreen    HexColor = 0x00FA9A
	b2_colorMediumTurquoise      HexColor = 0x48D1CC
	b2_colorMediumVioletRed      HexColor = 0xC71585
	b2_colorMidnightBlue         HexColor = 0x191970
	b2_colorMintCream            HexColor = 0xF5FFFA
	b2_colorMistyRose            HexColor = 0xFFE4E1
	b2_colorMoccasin             HexColor = 0xFFE4B5
	b2_colorNavajoWhite          HexColor = 0xFFDEAD
	b2_colorNavy                 HexColor = 0x000080
	b2_colorOldLace              HexColor = 0xFDF5E6
	b2_colorOlive                HexColor = 0x808000
	b2_colorOliveDrab            HexColor = 0x6B8E23
	b2_colorOrange               HexColor = 0xFFA500
	b2_colorOrangeRed            HexColor = 0xFF4500
	b2_colorOrchid               HexColor = 0xDA70D6
	b2_colorPaleGoldenRod        HexColor = 0xEEE8AA
	b2_colorPaleGreen            HexColor = 0x98FB98
	b2_colorPaleTurquoise        HexColor = 0xAFEEEE
	b2_colorPaleVioletRed        HexColor = 0xDB7093
	b2_colorPapayaWhip           HexColor = 0xFFEFD5
	b2_colorPeachPuff            HexColor = 0xFFDAB9
	b2_colorPeru                 HexColor = 0xCD853F
	b2_colorPink                 HexColor = 0xFFC0CB
	b2_colorPlum                 HexColor = 0xDDA0DD
	b2_colorPowderBlue           HexColor = 0xB0E0E6
	b2_colorPurple               HexColor = 0x800080
	b2_colorRebeccaPurple        HexColor = 0x663399
	b2_colorRed                  HexColor = 0xFF0000
	b2_colorRosyBrown            HexColor = 0xBC8F8F
	b2_colorRoyalBlue            HexColor = 0x4169E1
	b2_colorSaddleBrown          HexColor = 0x8B4513
	b2_colorSalmon               HexColor = 0xFA8072
	b2_colorSandyBrown           HexColor = 0xF4A460
	b2_colorSeaGreen             HexColor = 0x2E8B57
	b2_colorSeaShell             HexColor = 0xFFF5EE
	b2_colorSienna               HexColor = 0xA0522D
	b2_colorSilver               HexColor = 0xC0C0C0
	b2_colorSkyBlue              HexColor = 0x87CEEB
	b2_colorSlateBlue            HexColor = 0x6A5ACD
	b2_colorSlateGray            HexColor = 0x708090
	b2_colorSnow                 HexColor = 0xFFFAFA
	b2_colorSpringGreen          HexColor = 0x00FF7F
	b2_colorSteelBlue            HexColor = 0x4682B4
	b2_colorTan                  HexColor = 0xD2B48C
	b2_colorTeal                 HexColor = 0x008080
	b2_colorThistle              HexColor = 0xD8BFD8
	b2_colorTomato               HexColor = 0xFF6347
	b2_colorTurquoise            HexColor = 0x40E0D0
	b2_colorViolet               HexColor = 0xEE82EE
	b2_colorWheat                HexColor = 0xF5DEB3
	b2_colorWhite                HexColor = 0xFFFFFF
	b2_colorWhiteSmoke           HexColor = 0xF5F5F5
	b2_colorYellow               HexColor = 0xFFFF00
	b2_colorYellowGreen          HexColor = 0x9ACD32

	b2_colorBox2DRed    HexColor = 0xDC3132
	b2_colorBox2DBlue   HexColor = 0x30AEBF
	b2_colorBox2DGreen  HexColor = 0x8CC924
	b2_colorBox2DYellow HexColor = 0xFFEE8C
)

// Get the visualization color assigned to a constraint graph color slot. The last index
// (B2_GRAPH_COLOR_COUNT - 1) is the overflow color.
func GetGraphColor(index int) HexColor {
	return HexColor(C.b2GetGraphColor(C.int(index)))
}

// This struct holds callbacks you can implement to draw a Box2D world.
// This structure should be zero initialized.
// @ingroup world
type DebugDraw struct {
	// Draw a closed polygon provided in CCW order.
	DrawPolygonFcn func(vertices *Vec2, vertexCount int, color HexColor, context any)

	// Draw a solid closed polygon provided in CCW order.
	DrawSolidPolygonFcn func(transform Transform, vertices *Vec2, vertexCount int, radius float32, color HexColor, context any)

	// Draw a circle.
	DrawCircleFcn func(center Vec2, radius float32, color HexColor, context any)

	// Draw a solid circle.
	DrawSolidCircleFcn func(transform Transform, radius float32, color HexColor, context any)

	// Draw a solid capsule.
	DrawSolidCapsuleFcn func(p1 Vec2, p2 Vec2, radius float32, color HexColor, context any)

	// Draw a line segment.
	DrawLineFcn func(p1 Vec2, p2 Vec2, color HexColor, context any)

	// Draw a transform. Choose your own length scale.
	DrawTransformFcn func(transform Transform, context any)

	// Draw a point.
	DrawPointFcn func(p Vec2, size float32, color HexColor, context any)

	// Draw a string in world space
	DrawStringFcn func(p Vec2, s *byte, color HexColor, context any)

	// World bounds to use for debug draw
	DrawingBounds AABB

	// Scale to use when drawing forces
	ForceScale float32

	// Global scaling for joint drawing
	JointScale float32

	// Option to draw contact points
	DrawContacts bool

	// Draw anchor A for contact points (instead of anchorB)
	DrawAnchorA bool

	// Option to draw shapes
	DrawShapes bool

	// Option to draw joints
	DrawJoints bool

	// Option to draw additional information for joints
	DrawJointExtras bool

	// Option to draw the bounding boxes for shapes
	DrawBounds bool

	// Option to draw the mass and center of mass of dynamic bodies
	DrawMass bool

	// Option to draw body names
	DrawBodyNames bool

	// Option to visualize the graph coloring used for contacts and joints
	DrawGraphColors bool

	// Option to draw contact feature ids
	DrawContactFeatures bool

	// Option to draw contact normals
	DrawContactNormals bool

	// Option to draw contact normal forces
	DrawContactForces bool

	// Option to draw contact friction forces
	DrawFrictionForces bool

	// Option to draw islands as bounding boxes
	DrawIslands bool

	// User context that is passed as an argument to drawing callback functions
	Context any
}

// Use this to initialize your drawing interface. This allows you to implement a sub-set
// of the drawing functions.
func DefaultDebugDraw() DebugDraw {
	r := C.b2DefaultDebugDraw()
	return *cast[DebugDraw](&r)
}
