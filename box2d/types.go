package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

import (
	"unsafe"

	"github.com/Mishka-Squat/gamemath/vector2"
)

type Vec2 = vector2.Float32
type Rot = vector2.Float32

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

func goworlddefptr(ptr *C.b2WorldDef) *WorldDef {
	return (*WorldDef)(unsafe.Pointer(ptr))
}

func cworlddefptr(col *WorldDef) *C.b2WorldDef {
	return (*C.b2WorldDef)(unsafe.Pointer(col))
}

// / The body simulation type.
// / Each body is one of these three types. The type determines how the body behaves in the simulation.
// / @ingroup body
type BodyType int

const (
	/// zero mass, zero velocity, may be manually moved
	StaticBody BodyType = iota

	/// zero mass, velocity set by user, moved by solver
	KinematicBody

	/// positive mass, velocity determined by forces, moved by solver
	DynamicBody

	/// number of body types
	BodyTypeCount
)

// / Motion locks to restrict the body movement
type MotionLocks struct {
	/// Prevent translation along the x-axis
	LinearX bool

	/// Prevent translation along the y-axis
	LinearY bool

	/// Prevent rotation around the z-axis
	AngularZ bool
}

// / A body definition holds all the data needed to construct a rigid body.
// / You can safely re-use body definitions. Shapes are added to a body after construction.
// / Body definitions are temporary objects used to bundle creation parameters.
// / Must be initialized using b2DefaultBodyDef().
// / @ingroup body
type BodyDef struct {
	/// The body type: static, kinematic, or dynamic.
	Type BodyType

	/// The initial world position of the body. Bodies should be created with the desired position.
	/// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	/// if the body is moved after shapes have been added.
	Position Vec2

	/// The initial world rotation of the body. Use b2MakeRot() if you have an angle.
	Rotation Rot

	/// The initial linear velocity of the body's origin. Usually in meters per second.
	LinearVelocity Vec2

	/// The initial angular velocity of the body. Radians per second.
	AngularVelocity float32

	/// Linear damping is used to reduce the linear velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Generally linear damping is undesirable because it makes objects move slowly
	/// as if they are floating.
	LinearDamping float32

	/// Angular damping is used to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Angular damping can be use slow down rotating bodies.
	AngularDamping float32

	/// Scale the gravity applied to this body. Non-dimensional.
	GravityScale float32

	/// Sleep speed threshold, default is 0.05 meters per second
	SleepThreshold float32

	/// Optional body name for debugging. Up to 31 characters (excluding null termination)
	Name *byte

	/// Use this to store application specific body data.
	UserData any

	/// Motions locks to restrict linear and angular movement.
	/// Caution: may lead to softer constraints along the locked direction
	MotionLocks MotionLocks

	/// Set this flag to false if this body should never fall asleep.
	EnableSleep bool

	/// Is this body initially awake or sleeping?
	IsAwake bool

	/// Treat this body as a high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	/// @warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	/// continuous collision. They do not guarantee accurate collision if both bodies are fast moving because
	/// the bullet does a continuous check after all non-bullet bodies have moved. You could get unlucky and have
	/// the bullet body end a time step very close to a non-bullet body and the non-bullet body then moves over
	/// the bullet body. In continuous collision, initial overlap is ignored to avoid freezing bodies in place.
	/// I do not recommend using them for game projectiles if precise collision timing is needed. Instead consider
	/// using a ray or shape cast. You can use a marching ray or shape cast for projectile that moves over time.
	/// If you want a fast moving projectile to collide with a fast moving target, you need to consider the relative
	/// movement in your ray or shape cast. This is out of the scope of Box2D.
	/// So what are good use cases for bullets? Pinball games or games with dynamic containers that hold other objects.
	/// It should be a use case where it doesn't break the game if there is a collision missed, but having them
	/// captured improves the quality of the game.
	IsBullet bool

	/// Used to disable a body. A disabled body does not move or collide.
	IsEnabled bool

	/// This allows this body to bypass rotational speed limits. Should only be used
	/// for circular objects, like wheels.
	AllowFastRotation bool

	/// Used internally to detect a valid definition. DO NOT SET.
	InternalValue int
}

func gobodydefptr(ptr *C.b2BodyDef) *BodyDef {
	return (*BodyDef)(unsafe.Pointer(ptr))
}

func cbodydefptr(col *BodyDef) *C.b2BodyDef {
	return (*C.b2BodyDef)(unsafe.Pointer(col))
}
