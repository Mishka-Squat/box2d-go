package box2d

/*
#include "box2d/box2d.h"

extern bool b2World_OverlapAABB_fcn( b2ShapeId shapeId, void* context );

#cgo noescape b2CreateBody
*/
import "C"
import (
	"runtime"
	"runtime/cgo"
	"unsafe"

	"github.com/Mishka-Squat/gamemath/aabb2"
	"github.com/Mishka-Squat/gamemath/vector2"
)

// World id references a world instance. This should be treated as an opaque handle.
type WorldId struct {
	index1     uint16
	generation uint16
}

// Create a world for rigid body simulation. A world contains bodies, shapes, and constraints. You make create
// up to 128 worlds. Each world is completely independent and may be simulated in parallel.
// @return the world id.
func CreateWorld(def *WorldDef) WorldId {
	cdef := cast[C.b2WorldDef](def)
	r := C.b2CreateWorld(cdef)
	return *cast[WorldId](&r)
}

func (w WorldId) Defer() {
	w.Destroy()
}

// Destroy a world
func (w WorldId) Destroy() {
	C.b2DestroyWorld(*cast[C.b2WorldId](&w))
}

// Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
// on the stack and pass it as a pointer.
// @code{.c}
// b2BodyDef bodyDef = b2DefaultBodyDef();
// b2BodyId myBodyId = b2CreateBody(myWorldId, &bodyDef);
// @endcode
// @warning This function is locked during callbacks.
func (w WorldId) CreateBody(def *BodyDef) BodyId {
	cworld := *cast[C.b2WorldId](&w)
	cdef := cast[C.b2BodyDef](def)

	if def.UserData != nil {
		var pinner runtime.Pinner
		pinner.Pin(def.UserData)
		defer pinner.Unpin()
	}

	r := C.b2CreateBody(cworld, cdef)
	return *cast[BodyId](&r)
}

// World id validation. Provides validation for up to 64K allocations.
func (w WorldId) IsValid() bool {
	return bool(C.b2World_IsValid(*cast[C.b2WorldId](&w)))
}

// Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
// @param worldId The world to simulate
// @param timeStep The amount of time to simulate, this should be a fixed number. Usually 1/60.
// @param subStepCount The number of sub-steps, increasing the sub-step count can increase accuracy. Usually 4.
func (w WorldId) Step(timeStep float32, subStepCount int) {
	C.b2World_Step(*cast[C.b2WorldId](&w), C.float(timeStep), C.int(subStepCount))
}

// Call this to draw shapes and other debug draw data
func (w WorldId) Draw(draw *DebugDraw) {
	C.b2World_Draw(*cast[C.b2WorldId](&w), cast[C.b2DebugDraw](draw))
}

// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
func (w WorldId) GetBodyEvents() BodyEvents {
	r := C.b2World_GetBodyEvents(*cast[C.b2WorldId](&w))
	return *cast[BodyEvents](&r)
}

// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
func (w WorldId) GetSensorEvents() SensorEvents {
	r := C.b2World_GetSensorEvents(*cast[C.b2WorldId](&w))
	return *cast[SensorEvents](&r)
}

// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
func (w WorldId) GetContactEvents() ContactEvents {
	r := C.b2World_GetContactEvents(*cast[C.b2WorldId](&w))
	return *cast[ContactEvents](&r)
}

// Get the joint events for the current time step. The event data is transient. Do not store a reference to this data.
func (w WorldId) GetJointEvents() JointEvents {
	r := C.b2World_GetJointEvents(*cast[C.b2WorldId](&w))
	return *cast[JointEvents](&r)
}

func (w WorldId) ComputeAABB(fn ...func(shape ShapeId)) AABB {
	aabb := aabb2.NanFloat32()

	w.OverlapAABB(AABB{
		A: vector2.MinFloat32(),
		B: vector2.MaxFloat32(),
	}, DefaultQueryFilter(), func(shape ShapeId, context any) bool {
		aabb = aabb.Union(shape.GetAABB())
		for _, f := range fn {
			f(shape)
		}

		return true
	}, nil)

	return aabb
}

var go_b2World_OverlapAABB_fcn OverlapResultFcn

//export b2World_OverlapAABB_fcn
func b2World_OverlapAABB_fcn(shapeId C.b2ShapeId, context unsafe.Pointer) C.bool {
	if go_b2World_OverlapAABB_fcn == nil {
		return false
	}

	return C.bool(go_b2World_OverlapAABB_fcn(*cast[ShapeId](&shapeId), any(context)))
}

// Overlap test for all shapes that *potentially* overlap the provided AABB
func (w WorldId) OverlapAABB(aabb AABB, filter QueryFilter, fcn OverlapResultFcn, context any) TreeStats {
	cid := *cast[C.b2WorldId](&w)
	caabb := *cast[C.b2AABB](&aabb)
	cfilter := *cast[C.b2QueryFilter](&filter)
	go_b2World_OverlapAABB_fcn = fcn
	cfcn := (*C.b2OverlapResultFcn)(C.b2World_OverlapAABB_fcn)
	var ccontext unsafe.Pointer
	if context != nil {
		hcontext := cgo.NewHandle(context)
		defer hcontext.Delete()
		ccontext = unsafe.Pointer(hcontext)
	}

	r := C.b2World_OverlapAABB(cid, caabb, cfilter, cfcn, ccontext)
	return *cast[TreeStats](&r)
}

// Overlap test for all shapes that overlap the provided shape proxy.
//func (id WorldId) OverlapShape(proxy *ShapeProxy, filter QueryFilter, fcn OverlapResultFcn, context any) TreeStats {
//
//}

/*
// Cast a ray into the world to collect shapes in the path of the ray.
// Your callback function controls whether you get the closest point, any point, or n-points.
// @note The callback function may receive shapes in any order
// @param worldId The world to cast the ray against
// @param origin The start point of the ray
// @param translation The translation of the ray from the start point to the end point
// @param filter Contains bit flags to filter unwanted shapes from the results
// @param fcn A user implemented callback function
// @param context A user context that is passed along to the callback function
//	@return traversal performance counters
B2_API b2TreeStats b2World_CastRay( b2WorldId worldId, Vec2 origin, Vec2 translation, b2QueryFilter filter,
									b2CastResultFcn* fcn, void* context );

// Cast a ray into the world to collect the closest hit. This is a convenience function. Ignores initial overlap.
// This is less general than b2World_CastRay() and does not allow for custom filtering.
B2_API b2RayResult b2World_CastRayClosest( b2WorldId worldId, Vec2 origin, Vec2 translation, b2QueryFilter filter );

// Cast a shape through the world. Similar to a cast ray except that a shape is cast instead of a point.
//	@see b2World_CastRay
B2_API b2TreeStats b2World_CastShape( b2WorldId worldId, const b2ShapeProxy* proxy, Vec2 translation, b2QueryFilter filter,
									  b2CastResultFcn* fcn, void* context );

// Cast a capsule mover through the world. This is a special shape cast that handles sliding along other shapes while reducing
// clipping.
B2_API float32 b2World_CastMover( b2WorldId worldId, const Capsule* mover, Vec2 translation, b2QueryFilter filter );

// Collide a capsule mover with the world, gathering collision planes that can be fed to b2SolvePlanes. Useful for
// kinematic character movement.
B2_API void b2World_CollideMover( b2WorldId worldId, const Capsule* mover, b2QueryFilter filter, b2PlaneResultFcn* fcn,
								  void* context );
*/
// Enable/disable sleep. If your application does not need sleeping, you can gain some performance
// by disabling sleep completely at the world level.
// @see b2WorldDef
func (w WorldId) EnableSleeping(flag bool) {
	C.b2World_EnableSleeping(*cast[C.b2WorldId](&w), C.bool(flag))
}

// Is body sleeping enabled?
func (w WorldId) IsSleepingEnabled() bool {
	return bool(C.b2World_IsSleepingEnabled(*cast[C.b2WorldId](&w)))
}

// Enable/disable continuous collision between dynamic and static bodies. Generally you should keep continuous
// collision enabled to prevent fast moving objects from going through static objects. The performance gain from
// disabling continuous collision is minor.
// @see b2WorldDef
func (w WorldId) EnableContinuous(flag bool) {
	C.b2World_EnableContinuous(*cast[C.b2WorldId](&w), C.bool(flag))
}

// Is continuous collision enabled?
func (w WorldId) IsContinuousEnabled() bool {
	return bool(C.b2World_IsContinuousEnabled(*cast[C.b2WorldId](&w)))
}

// Adjust the restitution threshold. It is recommended not to make this value very small
// because it will prevent bodies from sleeping. Usually in meters per second.
// @see b2WorldDef
func (w WorldId) SetRestitutionThreshold(value float32) {
	C.b2World_SetRestitutionThreshold(*cast[C.b2WorldId](&w), C.float(value))
}

// Get the the restitution speed threshold. Usually in meters per second.
func (w WorldId) GetRestitutionThreshold() float32 {
	return float32(C.b2World_GetRestitutionThreshold(*cast[C.b2WorldId](&w)))
}

// Adjust the hit event threshold. This controls the collision speed needed to generate a b2ContactHitEvent.
// Usually in meters per second.
// @see b2WorldDef::hitEventThreshold
func (w WorldId) SetHitEventThreshold(value float32) {
	C.b2World_SetHitEventThreshold(*cast[C.b2WorldId](&w), C.float(value))
}

// Get the the hit event speed threshold. Usually in meters per second.
func (w WorldId) GetHitEventThreshold() float32 {
	return float32(C.b2World_GetHitEventThreshold(*cast[C.b2WorldId](&w)))
}

// Register the custom filter callback. This is optional.
func (w WorldId) SetCustomFilterCallback(fcn CustomFilterFcn, context any) {
	C.b2World_SetCustomFilterCallback(*cast[C.b2WorldId](&w), nil, nil)
}

// Register the pre-solve callback. This is optional.
func (w WorldId) SetPreSolveCallback(fcn PreSolveFcn, context any) {
	C.b2World_SetPreSolveCallback(*cast[C.b2WorldId](&w), nil, nil)
}

// Set the gravity vector for the entire world. Box2D has no concept of an up direction and this
// is left as a decision for the application. Usually in m/s^2.
// @see b2WorldDef
func (w WorldId) SetGravity(gravity Vec2) {
	C.b2World_SetGravity(*cast[C.b2WorldId](&w), *cast[C.b2Vec2](&gravity))
}

// Get the gravity vector
func (w WorldId) GetGravity() Vec2 {
	r := C.b2World_GetGravity(*cast[C.b2WorldId](&w))
	return *cast[Vec2](&r)
}

// Apply a radial explosion
// @param worldId The world id
// @param explosionDef The explosion definition
func (w WorldId) Explode(explosionDef *ExplosionDef) {
	C.b2World_Explode(*cast[C.b2WorldId](&w), cast[C.b2ExplosionDef](explosionDef))
}

// Adjust contact tuning parameters
// @param worldId The world id
// @param hertz The contact stiffness (cycles per second)
// @param dampingRatio The contact bounciness with 1 being critical damping (non-dimensional)
// @param pushSpeed The maximum contact constraint push out speed (meters per second)
// @note Advanced feature
func (w WorldId) SetContactTuning(hertz float32, dampingRatio float32, pushSpeed float32) {
	C.b2World_SetContactTuning(*cast[C.b2WorldId](&w), C.float(hertz), C.float(dampingRatio), C.float(pushSpeed))
}

// Set the contact point recycling distance. Setting this to zero disables contact point recycling.
// Usually in meters.
func (w WorldId) SetContactRecycleDistance(recycleDistance float32) {
	C.b2World_SetContactRecycleDistance(*cast[C.b2WorldId](&w), C.float(recycleDistance))
}

// Get the contact point recycling distance. Usually in meters.
func (w WorldId) GetContactRecycleDistance() float32 {
	return float32(C.b2World_GetContactRecycleDistance(*cast[C.b2WorldId](&w)))
}

// Set the maximum linear speed. Usually in m/s.
func (w WorldId) SetMaximumLinearSpeed(maximumLinearSpeed float32) {
	C.b2World_SetMaximumLinearSpeed(*cast[C.b2WorldId](&w), C.float(maximumLinearSpeed))
}

// Get the maximum linear speed. Usually in m/s.
func (w WorldId) GetMaximumLinearSpeed() float32 {
	return float32(C.b2World_GetMaximumLinearSpeed(*cast[C.b2WorldId](&w)))
}

// Enable/disable constraint warm starting. Advanced feature for testing. Disabling
// warm starting greatly reduces stability and provides no performance gain.
func (w WorldId) EnableWarmStarting(flag bool) {
	C.b2World_EnableWarmStarting(*cast[C.b2WorldId](&w), C.bool(flag))
}

// Is constraint warm starting enabled?
func (w WorldId) IsWarmStartingEnabled() bool {
	return bool(C.b2World_IsWarmStartingEnabled(*cast[C.b2WorldId](&w)))
}

// Get the number of awake bodies.
func (w WorldId) GetAwakeBodyCount() int {
	return int(C.b2World_GetAwakeBodyCount(*cast[C.b2WorldId](&w)))
}

// Get the current world performance profile
func (w WorldId) GetProfile() Profile {
	r := C.b2World_GetProfile(*cast[C.b2WorldId](&w))
	return *cast[Profile](&r)
}

// Get world counters and sizes
func (w WorldId) GetCounters() Counters {
	r := C.b2World_GetCounters(*cast[C.b2WorldId](&w))
	return *cast[Counters](&r)
}

// Get max capacity. This can be used with b2WorldDef to avoid run-time allocations and copies
func (w WorldId) GetMaxCapacity() Capacity {
	r := C.b2World_GetMaxCapacity(*cast[C.b2WorldId](&w))
	return *cast[Capacity](&r)
}

/*
// Set the user data pointer.
func (worldId WorldId) SetUserData(userData any) {

}

// Get the user data pointer.
func (worldId WorldId) GetUserData() any {

}

// Set the friction callback. Passing NULL resets to default.
func (worldId WorldId) SetFrictionCallback(callback FrictionCallback) {

}

// Set the restitution callback. Passing NULL resets to default.
func (worldId WorldId) SetRestitutionCallback(callback RestitutionCallback) {

}
*/
// Set the worker count. Must be between in the range [1, B2_MAX_WORKERS]
func (w WorldId) SetWorkerCount(count int) {
	C.b2World_SetWorkerCount(*cast[C.b2WorldId](&w), C.int(count))
}

// Get the worker count.
func (w WorldId) GetWorkerCount() int {
	return int(C.b2World_GetWorkerCount(*cast[C.b2WorldId](&w)))
}

// Dump memory stats to box2d_memory.txt
func (w WorldId) DumpMemoryStats() {
	C.b2World_DumpMemoryStats(*cast[C.b2WorldId](&w))
}

// This is for internal testing
func (w WorldId) RebuildStaticTree() {
	C.b2World_RebuildStaticTree(*cast[C.b2WorldId](&w))
}

// This is for internal testing
func (w WorldId) EnableSpeculative(flag bool) {
	C.b2World_EnableSpeculative(*cast[C.b2WorldId](&w), C.bool(flag))
}
