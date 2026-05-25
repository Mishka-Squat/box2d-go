package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

import (
	"unsafe"

	"github.com/Mishka-Squat/gamemath/aabb2"
	"github.com/Mishka-Squat/gamemath/transform2"
	"github.com/Mishka-Squat/gamemath/vector2"
)

type Vec2 = vector2.Float32
type Rot = vector2.Float32
type AABB = aabb2.Float32
type Transform = transform2.Float32

// / A 2-by-2 Matrix
type Mat22 struct {
	/// columns
	cx, cy Vec2
}

// separation = dot(normal, point) - offset
type Plane struct {
	Normal Vec2
	Offset float32
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
	NodeVisits int32
	LeafVisits int32
	Hit        bool
}

// Optional world capacities that can be used to avoid run-time allocations.
// @see b2World_GetMaxCapacity
// @ingroup world
type Capacity struct {
	// Number of expected static shapes.
	StaticShapeCount int32

	// Number of expected dynamic and kinematic shapes.
	DynamicShapeCount int32

	// Number of expected static bodies.
	StaticBodyCount int32

	// Number of expected dynamic and kinematic bodies.
	DynamicBodyCount int32

	// Number of expected contacts.
	ContactCount int32
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
	WorkerCount int32

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
	internalValue int32
}

func DefaultWorldDef() WorldDef {
	r := C.b2DefaultWorldDef()
	return *cast[WorldDef](&r)
}

// The body simulation type.
// Each body is one of these three types. The type determines how the body behaves in the simulation.
// @ingroup body
type BodyType int32

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
	UserData unsafe.Pointer

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
	internalValue int32
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
	GroupIndex int32
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

// Shape type
// @ingroup shape
type ShapeType int32

const (
	// A circle with an offset
	CircleShape ShapeType = iota

	// A capsule is an extruded circle
	CapsuleShape

	// A line segment
	SegmentShape

	// A convex polygon
	PolygonShape

	// A line segment owned by a chain shape
	ChainSegmentShape

	// The number of shape types
	ShapeTypeCount
)

// Surface materials allow chain shapes to have per segment surface properties.
// @ingroup shape
type SurfaceMaterial struct {
	// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	Friction float32

	// The coefficient of restitution (bounce) usually in the range [0,1].
	// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	Restitution float32

	// The rolling resistance usually in the range [0,1].
	RollingResistance float32

	// The tangent speed for conveyor belts
	TangentSpeed float32

	// User material identifier. This is passed with query results and to friction and restitution
	// combining functions. It is not used internally.
	UserMaterialId uint64

	// Custom debug draw color.
	CustomColor uint32
}

// Use this to initialize your surface material
// @ingroup shape
func DefaultSurfaceMaterial() SurfaceMaterial {
	r := C.b2DefaultSurfaceMaterial()
	return *cast[SurfaceMaterial](&r)
}

// Used to create a shape.
// This is a temporary object used to bundle shape creation parameters. You may use
// the same shape definition to create multiple shapes.
// Must be initialized using DefaultShapeDef().
// @ingroup shape
type ShapeDef struct {
	// Use this to store application specific shape data.
	UserData unsafe.Pointer

	// The surface material for this shape.
	Material SurfaceMaterial

	// The density, usually in kg/m^2.
	// This is not part of the surface material because this is for the interior, which may have
	// other considerations, such as being hollow. For example a wood barrel may be hollow or full of water.
	Density float32

	// Collision filtering data.
	Filter Filter

	// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See b2WorldDef.
	EnableCustomFiltering bool

	// A sensor shape generates overlap events but never generates a collision response.
	// Sensors do not have continuous collision. Instead, use a ray or shape cast for those scenarios.
	// Sensors still contribute to the body mass if they have non-zero density.
	// @note Sensor events are disabled by default.
	// @see enableSensorEvents
	IsSensor bool

	// Enable sensor events for this shape. This applies to sensors and non-sensors. Both shapes involved must have this flag set to true.
	// False by default, even for sensors.
	EnableSensorEvents bool

	// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Only one shape involved needs this flag set to true.
	// Ignored for sensors. False by default.
	EnableContactEvents bool

	// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Only one shape involved needs this flag set to true.
	// Ignored for sensors. False by default.
	EnableHitEvents bool

	// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	// and must be carefully handled due to multithreading. Ignored for sensors.
	EnablePreSolveEvents bool

	// When shapes are created they will scan the environment for collision the next time step. This can significantly slow down
	// static body creation when there are many static shapes.
	// This is flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
	InvokeContactCreation bool

	// Should the body update the mass properties when this shape is created. Default is true.
	// Warning: if this is true, you MUST call b2Body_ApplyMassFromShapes before simulating the world.
	UpdateBodyMass bool

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int32
}

// Use this to initialize your shape definition
// @ingroup shape
func DefaultShapeDef() ShapeDef {
	r := C.b2DefaultShapeDef()
	return *cast[ShapeDef](&r)
}

/*
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
*/
//! @cond
// Profiling data. Times are in milliseconds.
type Profile struct {
	Step                float32
	Pairs               float32
	Collide             float32
	Solve               float32
	SolverSetup         float32
	Constraints         float32
	PrepareConstraints  float32
	IntegrateVelocities float32
	WarmStart           float32
	SolveImpulses       float32
	IntegratePositions  float32
	RelaxImpulses       float32
	ApplyRestitution    float32
	StoreImpulses       float32
	SplitIslands        float32
	Transforms          float32
	SensorHits          float32
	JointEvents         float32
	HitEvents           float32
	Refit               float32
	Bullets             float32
	SleepIslands        float32
	Sensors             float32
}

// Counters that give details of the simulation size.
type Counters struct {
	BodyCount        int32
	ShapeCount       int32
	ContactCount     int32
	JointCount       int32
	IslandCount      int32
	StackUsed        int32
	StaticTreeHeight int32
	TreeHeight       int32
	ByteCount        int32
	TaskCount        int32
	ColorCounts      [24]int32

	// Number of contacts touched by the collide pass (graph contacts + awake-set non-touching).
	AwakeContactCount int32

	// Number of contacts recycled in the most recent step.
	RecycledContactCount int32
}

//! @endcond
/*
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
*/
// The explosion definition is used to configure options for explosions. Explosions
// consider shape geometry when computing the impulse.
// @ingroup world
type ExplosionDef struct {
	// Mask bits to filter shapes
	MaskBits uint64

	// The center of the explosion in world space
	Position Vec2

	// The radius of the explosion
	Radius float32

	// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	Falloff float32

	// Impulse per unit length. This applies an impulse according to the shape perimeter that
	// is facing the explosion. Explosions only apply to circles, capsules, and polygons. This
	// may be negative for implosions.
	ImpulsePerLength float32
}

/*
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
	BeginCount int32

	// The number of end touch events
	EndCount int32
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
	BeginCount int32

	// Number of end touch events
	EndCount int32

	// Number of hit events
	HitCount int32
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
	UserData   unsafe.Pointer
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
	MoveCount int32
}

// Joint events report joints that are awake and have a force and/or torque exceeding the threshold
// The observed forces and torques are not returned for efficiency reasons.
type JointEvent struct {
	// The joint id
	JointId JointId

	// The user data from the joint for convenience
	UserData unsafe.Pointer
}

// Joint events are buffered in the world and are available
// as event arrays after the time step is complete.
// Note: this data becomes invalid if joints are destroyed
type JointEvents struct {
	// Array of events
	JointEvents *JointEvent

	// Number of events
	Count int32
}

// The contact data for two shapes. By convention the manifold normal points
// from shape A to shape B.
// @see b2Shape_GetContactData() and b2Body_GetContactData()
type ContactData struct {
	ContactId ContactId
	ShapeIdA  ShapeId
	ShapeIdB  ShapeId
	Manifold  Manifold
}

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
type CustomFilterFcn = func(shapeIdA ShapeId, shapeIdB ShapeId, context any)

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
type PreSolveFcn = func(shapeIdA ShapeId, shapeIdB ShapeId, point Vec2, normal Vec2, context any) bool

// Prototype callback for overlap queries.
// Called for each shape found in the query.
// @see b2World_OverlapABB
// @return false to terminate the query.
// @ingroup world
type OverlapResultFcn = func(shapeId ShapeId, context any) bool

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
type CastResultFcn = func(shapeId ShapeId, point Vec2, normal Vec2, fraction float32, context any) float32

// Used to collect collision planes for character movers.
// Return true to continue gathering planes.
type PlaneResultFcn = func(shapeId ShapeId, plane *PlaneResult, context any) bool

// These colors are used for debug draw and mostly match the named SVG colors.
// See https://www.rapidtables.com/web/color/index.html
// https://johndecember.com/html/spec/colorsvg.html
// https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg
type HexColor int32

const (
	ColorAliceBlue            HexColor = 0xF0F8FF
	ColorAntiqueWhite         HexColor = 0xFAEBD7
	ColorAqua                 HexColor = 0x00FFFF
	ColorAquamarine           HexColor = 0x7FFFD4
	ColorAzure                HexColor = 0xF0FFFF
	ColorBeige                HexColor = 0xF5F5DC
	ColorBisque               HexColor = 0xFFE4C4
	ColorBlack                HexColor = 0x000000
	ColorBlanchedAlmond       HexColor = 0xFFEBCD
	ColorBlue                 HexColor = 0x0000FF
	ColorBlueViolet           HexColor = 0x8A2BE2
	ColorBrown                HexColor = 0xA52A2A
	ColorBurlywood            HexColor = 0xDEB887
	ColorCadetBlue            HexColor = 0x5F9EA0
	ColorChartreuse           HexColor = 0x7FFF00
	ColorChocolate            HexColor = 0xD2691E
	ColorCoral                HexColor = 0xFF7F50
	ColorCornflowerBlue       HexColor = 0x6495ED
	ColorCornsilk             HexColor = 0xFFF8DC
	ColorCrimson              HexColor = 0xDC143C
	ColorCyan                 HexColor = 0x00FFFF
	ColorDarkBlue             HexColor = 0x00008B
	ColorDarkCyan             HexColor = 0x008B8B
	ColorDarkGoldenRod        HexColor = 0xB8860B
	ColorDarkGray             HexColor = 0xA9A9A9
	ColorDarkGreen            HexColor = 0x006400
	ColorDarkKhaki            HexColor = 0xBDB76B
	ColorDarkMagenta          HexColor = 0x8B008B
	ColorDarkOliveGreen       HexColor = 0x556B2F
	ColorDarkOrange           HexColor = 0xFF8C00
	ColorDarkOrchid           HexColor = 0x9932CC
	ColorDarkRed              HexColor = 0x8B0000
	ColorDarkSalmon           HexColor = 0xE9967A
	ColorDarkSeaGreen         HexColor = 0x8FBC8F
	ColorDarkSlateBlue        HexColor = 0x483D8B
	ColorDarkSlateGray        HexColor = 0x2F4F4F
	ColorDarkTurquoise        HexColor = 0x00CED1
	ColorDarkViolet           HexColor = 0x9400D3
	ColorDeepPink             HexColor = 0xFF1493
	ColorDeepSkyBlue          HexColor = 0x00BFFF
	ColorDimGray              HexColor = 0x696969
	ColorDodgerBlue           HexColor = 0x1E90FF
	ColorFireBrick            HexColor = 0xB22222
	ColorFloralWhite          HexColor = 0xFFFAF0
	ColorForestGreen          HexColor = 0x228B22
	ColorFuchsia              HexColor = 0xFF00FF
	ColorGainsboro            HexColor = 0xDCDCDC
	ColorGhostWhite           HexColor = 0xF8F8FF
	ColorGold                 HexColor = 0xFFD700
	ColorGoldenRod            HexColor = 0xDAA520
	ColorGray                 HexColor = 0x808080
	ColorGreen                HexColor = 0x008000
	ColorGreenYellow          HexColor = 0xADFF2F
	ColorHoneyDew             HexColor = 0xF0FFF0
	ColorHotPink              HexColor = 0xFF69B4
	ColorIndianRed            HexColor = 0xCD5C5C
	ColorIndigo               HexColor = 0x4B0082
	ColorIvory                HexColor = 0xFFFFF0
	ColorKhaki                HexColor = 0xF0E68C
	ColorLavender             HexColor = 0xE6E6FA
	ColorLavenderBlush        HexColor = 0xFFF0F5
	ColorLawnGreen            HexColor = 0x7CFC00
	ColorLemonChiffon         HexColor = 0xFFFACD
	ColorLightBlue            HexColor = 0xADD8E6
	ColorLightCoral           HexColor = 0xF08080
	ColorLightCyan            HexColor = 0xE0FFFF
	ColorLightGoldenRodYellow HexColor = 0xFAFAD2
	ColorLightGray            HexColor = 0xD3D3D3
	ColorLightGreen           HexColor = 0x90EE90
	ColorLightPink            HexColor = 0xFFB6C1
	ColorLightSalmon          HexColor = 0xFFA07A
	ColorLightSeaGreen        HexColor = 0x20B2AA
	ColorLightSkyBlue         HexColor = 0x87CEFA
	ColorLightSlateGray       HexColor = 0x778899
	ColorLightSteelBlue       HexColor = 0xB0C4DE
	ColorLightYellow          HexColor = 0xFFFFE0
	ColorLime                 HexColor = 0x00FF00
	ColorLimeGreen            HexColor = 0x32CD32
	ColorLinen                HexColor = 0xFAF0E6
	ColorMagenta              HexColor = 0xFF00FF
	ColorMaroon               HexColor = 0x800000
	ColorMediumAquaMarine     HexColor = 0x66CDAA
	ColorMediumBlue           HexColor = 0x0000CD
	ColorMediumOrchid         HexColor = 0xBA55D3
	ColorMediumPurple         HexColor = 0x9370DB
	ColorMediumSeaGreen       HexColor = 0x3CB371
	ColorMediumSlateBlue      HexColor = 0x7B68EE
	ColorMediumSpringGreen    HexColor = 0x00FA9A
	ColorMediumTurquoise      HexColor = 0x48D1CC
	ColorMediumVioletRed      HexColor = 0xC71585
	ColorMidnightBlue         HexColor = 0x191970
	ColorMintCream            HexColor = 0xF5FFFA
	ColorMistyRose            HexColor = 0xFFE4E1
	ColorMoccasin             HexColor = 0xFFE4B5
	ColorNavajoWhite          HexColor = 0xFFDEAD
	ColorNavy                 HexColor = 0x000080
	ColorOldLace              HexColor = 0xFDF5E6
	ColorOlive                HexColor = 0x808000
	ColorOliveDrab            HexColor = 0x6B8E23
	ColorOrange               HexColor = 0xFFA500
	ColorOrangeRed            HexColor = 0xFF4500
	ColorOrchid               HexColor = 0xDA70D6
	ColorPaleGoldenRod        HexColor = 0xEEE8AA
	ColorPaleGreen            HexColor = 0x98FB98
	ColorPaleTurquoise        HexColor = 0xAFEEEE
	ColorPaleVioletRed        HexColor = 0xDB7093
	ColorPapayaWhip           HexColor = 0xFFEFD5
	ColorPeachPuff            HexColor = 0xFFDAB9
	ColorPeru                 HexColor = 0xCD853F
	ColorPink                 HexColor = 0xFFC0CB
	ColorPlum                 HexColor = 0xDDA0DD
	ColorPowderBlue           HexColor = 0xB0E0E6
	ColorPurple               HexColor = 0x800080
	ColorRebeccaPurple        HexColor = 0x663399
	ColorRed                  HexColor = 0xFF0000
	ColorRosyBrown            HexColor = 0xBC8F8F
	ColorRoyalBlue            HexColor = 0x4169E1
	ColorSaddleBrown          HexColor = 0x8B4513
	ColorSalmon               HexColor = 0xFA8072
	ColorSandyBrown           HexColor = 0xF4A460
	ColorSeaGreen             HexColor = 0x2E8B57
	ColorSeaShell             HexColor = 0xFFF5EE
	ColorSienna               HexColor = 0xA0522D
	ColorSilver               HexColor = 0xC0C0C0
	ColorSkyBlue              HexColor = 0x87CEEB
	ColorSlateBlue            HexColor = 0x6A5ACD
	ColorSlateGray            HexColor = 0x708090
	ColorSnow                 HexColor = 0xFFFAFA
	ColorSpringGreen          HexColor = 0x00FF7F
	ColorSteelBlue            HexColor = 0x4682B4
	ColorTan                  HexColor = 0xD2B48C
	ColorTeal                 HexColor = 0x008080
	ColorThistle              HexColor = 0xD8BFD8
	ColorTomato               HexColor = 0xFF6347
	ColorTurquoise            HexColor = 0x40E0D0
	ColorViolet               HexColor = 0xEE82EE
	ColorWheat                HexColor = 0xF5DEB3
	ColorWhite                HexColor = 0xFFFFFF
	ColorWhiteSmoke           HexColor = 0xF5F5F5
	ColorYellow               HexColor = 0xFFFF00
	ColorYellowGreen          HexColor = 0x9ACD32

	ColorBox2DRed    HexColor = 0xDC3132
	ColorBox2DBlue   HexColor = 0x30AEBF
	ColorBox2DGreen  HexColor = 0x8CC924
	ColorBox2DYellow HexColor = 0xFFEE8C
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
	Context unsafe.Pointer
}

// Use this to initialize your drawing interface. This allows you to implement a sub-set
// of the drawing functions.
func DefaultDebugDraw() DebugDraw {
	r := C.b2DefaultDebugDraw()
	return *cast[DebugDraw](&r)
}
