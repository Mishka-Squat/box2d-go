package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

// Create a world for rigid body simulation. A world contains bodies, shapes, and constraints. You make create
// up to 128 worlds. Each world is completely independent and may be simulated in parallel.
// @return the world id.
func CreateWorld(def *WorldDef) WorldId {
	cdef := cast[C.b2WorldDef](def)
	r := C.b2CreateWorld(cdef)
	return *cast[WorldId](&r)
}

// Destroy a world
func DestroyWorld(id WorldId) {
	C.b2DestroyWorld(*cast[C.b2WorldId](&id))
}

// World id validation. Provides validation for up to 64K allocations.
func World_IsValid(id WorldId) bool {
	return bool(C.b2World_IsValid(*cast[C.b2WorldId](&id)))
}

// Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
// @param worldId The world to simulate
// @param timeStep The amount of time to simulate, this should be a fixed number. Usually 1/60.
// @param subStepCount The number of sub-steps, increasing the sub-step count can increase accuracy. Usually 4.
func World_Step(id WorldId, timeStep float32, subStepCount int) {
	C.b2World_Step(*cast[C.b2WorldId](&id), C.float(timeStep), C.int(subStepCount))
}

// Call this to draw shapes and other debug draw data
func World_Draw(id WorldId, draw *DebugDraw) {
	C.b2World_Draw(*cast[C.b2WorldId](&id), cast[C.b2DebugDraw](draw))
}

// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
func World_GetBodyEvents(id WorldId) BodyEvents {
	r := C.b2World_GetBodyEvents(*cast[C.b2WorldId](&id))
	return *cast[BodyEvents](&r)
}

// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
func World_GetSensorEvents(id WorldId) SensorEvents {
	r := C.b2World_GetSensorEvents(*cast[C.b2WorldId](&id))
	return *cast[SensorEvents](&r)
}

// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
func World_GetContactEvents(id WorldId) ContactEvents {
	r := C.b2World_GetContactEvents(*cast[C.b2WorldId](&id))
	return *cast[ContactEvents](&r)
}

// Get the joint events for the current time step. The event data is transient. Do not store a reference to this data.
func World_GetJointEvents(id WorldId) JointEvents {
	r := C.b2World_GetJointEvents(*cast[C.b2WorldId](&id))
	return *cast[JointEvents](&r)
}

// https://stackoverflow.com/questions/37157379/passing-function-pointer-to-the-c-code-using-cgo
// Overlap test for all shapes that *potentially* overlap the provided AABB
func World_OverlapAABB(id WorldId, aabb AABB, filter QueryFilter, fcn OverlapResultFcn, context any) TreeStats {
	cid := *cast[C.b2WorldId](&id)
	caabb := *cast[C.b2AABB](&aabb)
	cfilter := *cast[C.b2QueryFilter](&filter)
	//cfcn := *cast[C.b2OverlapResultFcn](fcn)
	r := C.b2World_OverlapAABB(cid, caabb, cfilter, nil, nil)
	return *cast[TreeStats](&r)
}

// Overlap test for all shapes that overlap the provided shape proxy.
//func World_OverlapShape(id WorldId, proxy *ShapeProxy, filter QueryFilter, fcn OverlapResultFcn, context any) TreeStats {
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
func World_EnableSleeping(worldId WorldId, flag bool) {
	C.b2World_EnableSleeping(*cast[C.b2WorldId](&worldId), C.bool(flag))
}

// Is body sleeping enabled?
func World_IsSleepingEnabled(worldId WorldId) bool {
	return bool(C.b2World_IsSleepingEnabled(*cast[C.b2WorldId](&worldId)))
}

// Enable/disable continuous collision between dynamic and static bodies. Generally you should keep continuous
// collision enabled to prevent fast moving objects from going through static objects. The performance gain from
// disabling continuous collision is minor.
// @see b2WorldDef
func World_EnableContinuous(worldId WorldId, flag bool) {
	C.b2World_EnableContinuous(*cast[C.b2WorldId](&worldId), C.bool(flag))
}

// Is continuous collision enabled?
func World_IsContinuousEnabled(worldId WorldId) bool {
	return bool(C.b2World_IsContinuousEnabled(*cast[C.b2WorldId](&worldId)))
}

// Adjust the restitution threshold. It is recommended not to make this value very small
// because it will prevent bodies from sleeping. Usually in meters per second.
// @see b2WorldDef
func World_SetRestitutionThreshold(worldId WorldId, value float32) {
	C.b2World_SetRestitutionThreshold(*cast[C.b2WorldId](&worldId), C.float(value))
}

// Get the the restitution speed threshold. Usually in meters per second.
func World_GetRestitutionThreshold(worldId WorldId) float32 {
	return float32(C.b2World_GetRestitutionThreshold(*cast[C.b2WorldId](&worldId)))
}

// Adjust the hit event threshold. This controls the collision speed needed to generate a b2ContactHitEvent.
// Usually in meters per second.
// @see b2WorldDef::hitEventThreshold
func World_SetHitEventThreshold(worldId WorldId, value float32) {
	C.b2World_SetHitEventThreshold(*cast[C.b2WorldId](&worldId), C.float(value))
}

// Get the the hit event speed threshold. Usually in meters per second.
func World_GetHitEventThreshold(worldId WorldId) float32 {
	return float32(C.b2World_GetHitEventThreshold(*cast[C.b2WorldId](&worldId)))
}

// Register the custom filter callback. This is optional.
func World_SetCustomFilterCallback(worldId WorldId, fcn b2CustomFilterFcn, context any) {
	C.b2World_SetCustomFilterCallback(*cast[C.b2WorldId](&worldId), nil, nil)
}

// Register the pre-solve callback. This is optional.
func World_SetPreSolveCallback(worldId WorldId, fcn b2PreSolveFcn, context any) {
	C.b2World_SetPreSolveCallback(*cast[C.b2WorldId](&worldId), nil, nil)
}

// Set the gravity vector for the entire world. Box2D has no concept of an up direction and this
// is left as a decision for the application. Usually in m/s^2.
// @see b2WorldDef
func World_SetGravity(worldId WorldId, gravity Vec2) {
	C.b2World_SetGravity(*cast[C.b2WorldId](&worldId), *cast[C.b2Vec2](&gravity))
}

// Get the gravity vector
func World_GetGravity(worldId WorldId) Vec2 {
	r := C.b2World_GetGravity(*cast[C.b2WorldId](&worldId))
	return *cast[Vec2](&r)
}

// Apply a radial explosion
// @param worldId The world id
// @param explosionDef The explosion definition
func World_Explode(worldId WorldId, explosionDef *ExplosionDef) {
	C.b2World_Explode(*cast[C.b2WorldId](&worldId), cast[C.b2ExplosionDef](explosionDef))
}

// Adjust contact tuning parameters
// @param worldId The world id
// @param hertz The contact stiffness (cycles per second)
// @param dampingRatio The contact bounciness with 1 being critical damping (non-dimensional)
// @param pushSpeed The maximum contact constraint push out speed (meters per second)
// @note Advanced feature
func World_SetContactTuning(worldId WorldId, hertz float32, dampingRatio float32, pushSpeed float32) {
	C.b2World_SetContactTuning(*cast[C.b2WorldId](&worldId), C.float(hertz), C.float(dampingRatio), C.float(pushSpeed))
}

// Set the contact point recycling distance. Setting this to zero disables contact point recycling.
// Usually in meters.
func World_SetContactRecycleDistance(worldId WorldId, recycleDistance float32) {
	C.b2World_SetContactRecycleDistance(*cast[C.b2WorldId](&worldId), C.float(recycleDistance))
}

// Get the contact point recycling distance. Usually in meters.
func World_GetContactRecycleDistance(worldId WorldId) float32 {
	return float32(C.b2World_GetContactRecycleDistance(*cast[C.b2WorldId](&worldId)))
}

// Set the maximum linear speed. Usually in m/s.
func World_SetMaximumLinearSpeed(worldId WorldId, maximumLinearSpeed float32) {
	C.b2World_SetMaximumLinearSpeed(*cast[C.b2WorldId](&worldId), C.float(maximumLinearSpeed))
}

// Get the maximum linear speed. Usually in m/s.
func World_GetMaximumLinearSpeed(worldId WorldId) float32 {
	return float32(C.b2World_GetMaximumLinearSpeed(*cast[C.b2WorldId](&worldId)))
}

// Enable/disable constraint warm starting. Advanced feature for testing. Disabling
// warm starting greatly reduces stability and provides no performance gain.
func World_EnableWarmStarting(worldId WorldId, flag bool) {
	C.b2World_EnableWarmStarting(*cast[C.b2WorldId](&worldId), C.bool(flag))
}

// Is constraint warm starting enabled?
func World_IsWarmStartingEnabled(worldId WorldId) bool {
	return bool(C.b2World_IsWarmStartingEnabled(*cast[C.b2WorldId](&worldId)))
}

// Get the number of awake bodies.
func World_GetAwakeBodyCount(worldId WorldId) int {
	return int(C.b2World_GetAwakeBodyCount(*cast[C.b2WorldId](&worldId)))
}

// Get the current world performance profile
func World_GetProfile(worldId WorldId) Profile {
	r := C.b2World_GetProfile(*cast[C.b2WorldId](&worldId))
	return *cast[Profile](&r)
}

// Get world counters and sizes
func World_GetCounters(worldId WorldId) Counters {
	r := C.b2World_GetCounters(*cast[C.b2WorldId](&worldId))
	return *cast[Counters](&r)
}

// Get max capacity. This can be used with b2WorldDef to avoid run-time allocations and copies
func World_GetMaxCapacity(worldId WorldId) Capacity {
	r := C.b2World_GetMaxCapacity(*cast[C.b2WorldId](&worldId))
	return *cast[Capacity](&r)
}

/*
// Set the user data pointer.
func World_SetUserData(worldId WorldId, userData any) {

}

// Get the user data pointer.
func World_GetUserData(worldId WorldId) any {

}

// Set the friction callback. Passing NULL resets to default.
func World_SetFrictionCallback(worldId WorldId, callback FrictionCallback) {

}

// Set the restitution callback. Passing NULL resets to default.
func World_SetRestitutionCallback(worldId WorldId, callback RestitutionCallback) {

}
*/
// Set the worker count. Must be between in the range [1, B2_MAX_WORKERS]
func World_SetWorkerCount(worldId WorldId, count int) {
	C.b2World_SetWorkerCount(*cast[C.b2WorldId](&worldId), C.int(count))
}

// Get the worker count.
func World_GetWorkerCount(worldId WorldId) int {
	return int(C.b2World_GetWorkerCount(*cast[C.b2WorldId](&worldId)))
}

// Dump memory stats to box2d_memory.txt
func World_DumpMemoryStats(worldId WorldId) {
	C.b2World_DumpMemoryStats(*cast[C.b2WorldId](&worldId))
}

// This is for internal testing
func World_RebuildStaticTree(worldId WorldId) {
	C.b2World_RebuildStaticTree(*cast[C.b2WorldId](&worldId))
}

// This is for internal testing
func World_EnableSpeculative(worldId WorldId, flag bool) {
	C.b2World_EnableSpeculative(*cast[C.b2WorldId](&worldId), C.bool(flag))
}

// Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
// on the stack and pass it as a pointer.
// @code{.c}
// b2BodyDef bodyDef = b2DefaultBodyDef();
// b2BodyId myBodyId = b2CreateBody(myWorldId, &bodyDef);
// @endcode
// @warning This function is locked during callbacks.
func CreateBody(world WorldId, def *BodyDef) BodyId {
	cworld := *cast[C.b2WorldId](&world)
	cdef := cast[C.b2BodyDef](def)
	r := C.b2CreateBody(cworld, cdef)
	return *cast[BodyId](&r)
}

// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
// Do not keep references to the associated shapes and joints.
func DestroyBody(id BodyId) {
	C.b2DestroyBody(*cast[C.b2BodyId](&id))
}

// Body identifier validation. A valid body exists in a world and is non-null.
// This can be used to detect orphaned ids. Provides validation for up to 64K allocations.
func Body_IsValid(id BodyId) bool {
	return bool(C.b2Body_IsValid(*cast[C.b2BodyId](&id)))
}

// Get the body type: static, kinematic, or dynamic
func Body_GetType(bodyId BodyId) BodyType {
	return BodyType(C.b2Body_GetType(*cast[C.b2BodyId](&bodyId)))
}

// Change the body type. This is an expensive operation. This automatically updates the mass
// properties regardless of the automatic mass setting.
func Body_SetType(bodyId BodyId, _type BodyType) {
	C.b2Body_SetType(*cast[C.b2BodyId](&bodyId), C.b2BodyType(_type))
}

/*
// Set the body name. Up to 31 characters excluding 0 termination.
B2_API void b2Body_SetName( b2BodyId bodyId, const char* name );

// Get the body name.
B2_API const char* b2Body_GetName( b2BodyId bodyId );

// Set the user data for a body
B2_API void b2Body_SetUserData( b2BodyId bodyId, void* userData );

// Get the user data stored in a body
B2_API void* b2Body_GetUserData( b2BodyId bodyId );
*/
// Get the world position of a body. This is the location of the body origin.
func Body_GetPosition(bodyId BodyId) Vec2 {
	r := C.b2Body_GetPosition(*cast[C.b2BodyId](&bodyId))
	return *cast[Vec2](&r)
}

// Get the world rotation of a body as a cosine/sine pair (complex number)
func Body_GetRotation(bodyId BodyId) Rot {
	r := C.b2Body_GetRotation(*cast[C.b2BodyId](&bodyId))
	return *cast[Rot](&r)
}

// Get the world transform of a body.
func Body_GetTransform(bodyId BodyId) Transform {
	r := C.b2Body_GetTransform(*cast[C.b2BodyId](&bodyId))
	return *cast[Transform](&r)
}

// Set the world transform of a body. This acts as a teleport and is fairly expensive.
// @note Generally you should create a body with then intended transform.
// @see b2BodyDef::position and b2BodyDef::rotation
func Body_SetTransform(bodyId BodyId, position Vec2, rotation Rot) {
	C.b2Body_SetTransform(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&position), *cast[C.b2Rot](&rotation))
}

// Get a local point on a body given a world point
func Body_GetLocalPoint(bodyId BodyId, worldPoint Vec2) Vec2 {
	r := C.b2Body_GetLocalPoint(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&worldPoint))
	return *cast[Vec2](&r)
}

// Get a world point on a body given a local point
func Body_GetWorldPoint(bodyId BodyId, localPoint Vec2) Vec2 {
	r := C.b2Body_GetWorldPoint(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&localPoint))
	return *cast[Vec2](&r)
}

// Get a local vector on a body given a world vector
func Body_GetLocalVector(bodyId BodyId, worldVector Vec2) Vec2 {
	r := C.b2Body_GetLocalVector(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&worldVector))
	return *cast[Vec2](&r)
}

// Get a world vector on a body given a local vector
func Body_GetWorldVector(bodyId BodyId, localVector Vec2) Vec2 {
	r := C.b2Body_GetWorldVector(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&localVector))
	return *cast[Vec2](&r)
}

// Get the linear velocity of a body's center of mass. Usually in meters per second.
func Body_GetLinearVelocity(bodyId BodyId) Vec2 {
	r := C.b2Body_GetLinearVelocity(*cast[C.b2BodyId](&bodyId))
	return *cast[Vec2](&r)
}

// Get the angular velocity of a body in radians per second
func Body_GetAngularVelocity(bodyId BodyId) float32 {
	return float32(C.b2Body_GetAngularVelocity(*cast[C.b2BodyId](&bodyId)))
}

// Set the linear velocity of a body. Usually in meters per second.
func Body_SetLinearVelocity(bodyId BodyId, linearVelocity Vec2) {
	C.b2Body_SetLinearVelocity(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&linearVelocity))
}

// Set the angular velocity of a body in radians per second
func Body_SetAngularVelocity(bodyId BodyId, angularVelocity float32) {
	C.b2Body_SetAngularVelocity(*cast[C.b2BodyId](&bodyId), C.float(angularVelocity))
}

// Set the velocity to reach the given transform after a given time step.
// The result will be close but maybe not exact. This is meant for kinematic bodies.
// The target is not applied if the velocity would be below the sleep threshold and
// the body is currently asleep.
// @param bodyId The body id
// @param target The target transform for the body
// @param timeStep The time step of the next call to b2World_Step
// @param wake Option to wake the body or not
func Body_SetTargetTransform(bodyId BodyId, target Transform, timeStep float32, wake bool) {
	C.b2Body_SetTargetTransform(*cast[C.b2BodyId](&bodyId), *cast[C.b2Transform](&target), C.float(timeStep), C.bool(wake))
}

// Get the linear velocity of a local point attached to a body. Usually in meters per second.
func Body_GetLocalPointVelocity(bodyId BodyId, localPoint Vec2) Vec2 {
	r := C.b2Body_GetLocalPointVelocity(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&localPoint))
	return *cast[Vec2](&r)
}

// Get the linear velocity of a world point attached to a body. Usually in meters per second.
func Body_GetWorldPointVelocity(bodyId BodyId, worldPoint Vec2) Vec2 {
	r := C.b2Body_GetWorldPointVelocity(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&worldPoint))
	return *cast[Vec2](&r)
}

// Apply a force at a world point. If the force is not applied at the center of mass,
// it will generate a torque and affect the angular velocity. This optionally wakes up the body.
// The force is ignored if the body is not awake.
// @param bodyId The body id
// @param force The world force vector, usually in newtons (N)
// @param point The world position of the point of application
// @param wake Option to wake up the body
func Body_ApplyForce(bodyId BodyId, force Vec2, point Vec2, wake bool) {
	C.b2Body_ApplyForce(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&force), *cast[C.b2Vec2](&point), C.bool(wake))
}

// Apply a force to the center of mass. This optionally wakes up the body.
// The force is ignored if the body is not awake.
// @param bodyId The body id
// @param force the world force vector, usually in newtons (N).
// @param wake also wake up the body
func Body_ApplyForceToCenter(bodyId BodyId, force Vec2, wake bool) {
	C.b2Body_ApplyForceToCenter(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&force), C.bool(wake))
}

// Apply a torque. This affects the angular velocity without affecting the linear velocity.
// This optionally wakes the body. The torque is ignored if the body is not awake.
// @param bodyId The body id
// @param torque about the z-axis (out of the screen), usually in N*m.
// @param wake also wake up the body
func Body_ApplyTorque(bodyId BodyId, torque float32, wake bool) {
	C.b2Body_ApplyTorque(*cast[C.b2BodyId](&bodyId), C.float(torque), C.bool(wake))
}

// Clear the force and torque on this body. Forces and torques are automatically cleared after each world
// step. So this only needs to be called if the application wants to remove the effect of previous
// calls to apply forces and torques before the world step is called.
// @param bodyId The body id
func Body_ClearForces(bodyId BodyId) {
	C.b2Body_ClearForces(*cast[C.b2BodyId](&bodyId))
}

// Apply an impulse at a point. This immediately modifies the velocity.
// It also modifies the angular velocity if the point of application
// is not at the center of mass. This optionally wakes the body.
// The impulse is ignored if the body is not awake.
// @param bodyId The body id
// @param impulse the world impulse vector, usually in N*s or kg*m/s.
// @param point the world position of the point of application.
// @param wake also wake up the body
// @warning This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
func Body_ApplyLinearImpulse(bodyId BodyId, impulse Vec2, point Vec2, wake bool) {
	C.b2Body_ApplyLinearImpulse(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&impulse), *cast[C.b2Vec2](&point), C.bool(wake))
}

// Apply an impulse to the center of mass. This immediately modifies the velocity.
// The impulse is ignored if the body is not awake. This optionally wakes the body.
// @param bodyId The body id
// @param impulse the world impulse vector, usually in N*s or kg*m/s.
// @param wake also wake up the body
// @warning This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
func Body_ApplyLinearImpulseToCenter(bodyId BodyId, impulse Vec2, wake bool) {
	C.b2Body_ApplyLinearImpulseToCenter(*cast[C.b2BodyId](&bodyId), *cast[C.b2Vec2](&impulse), C.bool(wake))
}

// Apply an angular impulse. The impulse is ignored if the body is not awake.
// This optionally wakes the body.
// @param bodyId The body id
// @param impulse the angular impulse, usually in units of kg*m*m/s
// @param wake also wake up the body
// @warning This should be used for one-shot impulses. If you need a steady torque,
// use a torque instead, which will work better with the sub-stepping solver.
func Body_ApplyAngularImpulse(bodyId BodyId, impulse float32, wake bool) {
	C.b2Body_ApplyAngularImpulse(*cast[C.b2BodyId](&bodyId), C.float(impulse), C.bool(wake))
}

// Get the mass of the body, usually in kilograms
func Body_GetMass(bodyId BodyId) float32 {
	return float32(C.b2Body_GetMass(*cast[C.b2BodyId](&bodyId)))
}

// Get the rotational inertia of the body, usually in kg*m^2
func Body_GetRotationalInertia(bodyId BodyId) float32 {
	return float32(C.b2Body_GetRotationalInertia(*cast[C.b2BodyId](&bodyId)))
}

// Get the center of mass position of the body in local space
func Body_GetLocalCenterOfMass(bodyId BodyId) Vec2 {
	r := C.b2Body_GetLocalCenterOfMass(*cast[C.b2BodyId](&bodyId))
	return *cast[Vec2](&r)
}

// Get the center of mass position of the body in world space
func Body_GetWorldCenterOfMass(bodyId BodyId) Vec2 {
	r := C.b2Body_GetWorldCenterOfMass(*cast[C.b2BodyId](&bodyId))
	return *cast[Vec2](&r)
}

// Override the body's mass properties. Normally this is computed automatically using the
// shape geometry and density. This information is lost if a shape is added or removed or if the
// body type changes.
func Body_SetMassData(bodyId BodyId, massData MassData) {
	C.b2Body_SetMassData(*cast[C.b2BodyId](&bodyId), *cast[C.b2MassData](&massData))
}

// Get the mass data for a body
func Body_GetMassData(bodyId BodyId) MassData {
	r := C.b2Body_GetMassData(*cast[C.b2BodyId](&bodyId))
	return *cast[MassData](&r)
}

// This updates the mass properties to the sum of the mass properties of the shapes.
// This normally does not need to be called unless you called SetMassData to override
// the mass and you later want to reset the mass.
// You may also use this when automatic mass computation has been disabled.
// You should call this regardless of body type.
// Note that sensor shapes may have mass.
func Body_ApplyMassFromShapes(bodyId BodyId) {
	C.b2Body_ApplyMassFromShapes(*cast[C.b2BodyId](&bodyId))
}

// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
func Body_SetLinearDamping(bodyId BodyId, linearDamping float32) {
	C.b2Body_SetLinearDamping(*cast[C.b2BodyId](&bodyId), C.float(linearDamping))
}

// Get the current linear damping.
func Body_GetLinearDamping(bodyId BodyId) float32 {
	return float32(C.b2Body_GetLinearDamping(*cast[C.b2BodyId](&bodyId)))
}

// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
func Body_SetAngularDamping(bodyId BodyId, angularDamping float32) {
	C.b2Body_SetAngularDamping(*cast[C.b2BodyId](&bodyId), C.float(angularDamping))
}

// Get the current angular damping.
func Body_GetAngularDamping(bodyId BodyId) float32 {
	return float32(C.b2Body_GetAngularDamping(*cast[C.b2BodyId](&bodyId)))
}

// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
// @see b2BodyDef::gravityScale
func Body_SetGravityScale(bodyId BodyId, gravityScale float32) {
	C.b2Body_SetGravityScale(*cast[C.b2BodyId](&bodyId), C.float(gravityScale))
}

// Get the current gravity scale
func Body_GetGravityScale(bodyId BodyId) float32 {
	return float32(C.b2Body_GetGravityScale(*cast[C.b2BodyId](&bodyId)))
}

// @return true if this body is awake
func Body_IsAwake(bodyId BodyId) bool {
	return bool(C.b2Body_IsAwake(*cast[C.b2BodyId](&bodyId)))
}

// Wake a body from sleep. This wakes the entire island the body is touching.
// @warning Putting a body to sleep will put the entire island of bodies touching this body to sleep,
// which can be expensive and possibly unintuitive.
func Body_SetAwake(bodyId BodyId, awake bool) {
	C.b2Body_SetAwake(*cast[C.b2BodyId](&bodyId), C.bool(awake))
}

// Wake bodies touching this body. Works for static bodies.
func Body_WakeTouching(bodyId BodyId) {
	C.b2Body_WakeTouching(*cast[C.b2BodyId](&bodyId))
}

// Enable or disable sleeping for this body. If sleeping is disabled the body will wake.
func Body_EnableSleep(bodyId BodyId, enableSleep bool) {
	C.b2Body_EnableSleep(*cast[C.b2BodyId](&bodyId), C.bool(enableSleep))
}

// Returns true if sleeping is enabled for this body
func Body_IsSleepEnabled(bodyId BodyId) bool {
	return bool(C.b2Body_IsSleepEnabled(*cast[C.b2BodyId](&bodyId)))
}

// Set the sleep threshold, usually in meters per second
func Body_SetSleepThreshold(bodyId BodyId, sleepThreshold float32) {
	C.b2Body_SetSleepThreshold(*cast[C.b2BodyId](&bodyId), C.float(sleepThreshold))
}

// Get the sleep threshold, usually in meters per second.
func Body_GetSleepThreshold(bodyId BodyId) float32 {
	return float32(C.b2Body_GetSleepThreshold(*cast[C.b2BodyId](&bodyId)))
}

// Returns true if this body is enabled
func Body_IsEnabled(bodyId BodyId) bool {
	return bool(C.b2Body_IsEnabled(*cast[C.b2BodyId](&bodyId)))
}

// Disable a body by removing it completely from the simulation. This is expensive.
func Body_Disable(bodyId BodyId) {
	C.b2Body_Disable(*cast[C.b2BodyId](&bodyId))
}

// Enable a body by adding it to the simulation. This is expensive.
func Body_Enable(bodyId BodyId) {
	C.b2Body_Enable(*cast[C.b2BodyId](&bodyId))
}

// Set the motion locks on this body.
func Body_SetMotionLocks(bodyId BodyId, locks MotionLocks) {
	C.b2Body_SetMotionLocks(*cast[C.b2BodyId](&bodyId), *cast[C.b2MotionLocks](&locks))
}

// Get the motion locks for this body.
func Body_GetMotionLocks(bodyId BodyId) MotionLocks {
	r := C.b2Body_GetMotionLocks(*cast[C.b2BodyId](&bodyId))
	return *cast[MotionLocks](&r)
}

// Set this body to be a bullet. A bullet does continuous collision detection
// against dynamic bodies (but not other bullets).
func Body_SetBullet(bodyId BodyId, flag bool) {
	C.b2Body_SetBullet(*cast[C.b2BodyId](&bodyId), C.bool(flag))
}

// Is this body a bullet?
func Body_IsBullet(bodyId BodyId) bool {
	return bool(C.b2Body_IsBullet(*cast[C.b2BodyId](&bodyId)))
}

// Enable/disable contact events on all shapes.
// @see b2ShapeDef::enableContactEvents
// @warning changing this at runtime may cause mismatched begin/end touch events
func Body_EnableContactEvents(bodyId BodyId, flag bool) {
	C.b2Body_EnableContactEvents(*cast[C.b2BodyId](&bodyId), C.bool(flag))
}

// Enable/disable hit events on all shapes
// @see b2ShapeDef::enableHitEvents
func Body_EnableHitEvents(bodyId BodyId, flag bool) {
	C.b2Body_EnableHitEvents(*cast[C.b2BodyId](&bodyId), C.bool(flag))
}

// Get the world that owns this body
func Body_GetWorld(bodyId BodyId) WorldId {
	r := C.b2Body_GetWorld(*cast[C.b2BodyId](&bodyId))
	return *cast[WorldId](&r)
}

// Get the number of shapes on this body
func Body_GetShapeCount(bodyId BodyId) int {
	return int(C.b2Body_GetShapeCount(*cast[C.b2BodyId](&bodyId)))
}

// Get the shape ids for all shapes on this body, up to the provided capacity.
// @returns the number of shape ids stored in the user array
func Body_GetShapes(bodyId BodyId, shapeArray *ShapeId, capacity int) int {
	return int(C.b2Body_GetShapes(*cast[C.b2BodyId](&bodyId), cast[C.b2ShapeId](shapeArray), C.int(capacity)))
}

// Get the number of joints on this body
func Body_GetJointCount(bodyId BodyId) int {
	return int(C.b2Body_GetJointCount(*cast[C.b2BodyId](&bodyId)))
}

// Get the joint ids for all joints on this body, up to the provided capacity
// @returns the number of joint ids stored in the user array
func Body_GetJoints(bodyId BodyId, jointArray *JointId, capacity int) int {
	return int(C.b2Body_GetJoints(*cast[C.b2BodyId](&bodyId), cast[C.b2JointId](jointArray), C.int(capacity)))
}

// Get the maximum capacity required for retrieving all the touching contacts on a body
func Body_GetContactCapacity(bodyId BodyId) int {
	return int(C.b2Body_GetContactCapacity(*cast[C.b2BodyId](&bodyId)))
}

// Get the touching contact data for a body.
// @note Box2D uses speculative collision so some contact points may be separated.
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
func Body_GetContactData(bodyId BodyId, contactData *ContactData, capacity int) int {
	return int(C.b2Body_GetContactData(*cast[C.b2BodyId](&bodyId), cast[C.b2ContactData](contactData), C.int(capacity)))
}

// Get the current world AABB that contains all the attached shapes. Note that this may not encompass the body origin.
// If there are no shapes attached then the returned AABB is empty and centered on the body origin.
func Body_ComputeAABB(bodyId BodyId) AABB {
	r := C.b2Body_ComputeAABB(*cast[C.b2BodyId](&bodyId))
	return *cast[AABB](&r)
}

//
// @defgroup shape Shape
// Functions to create, destroy, and access.
// Shapes bind raw geometry to bodies and hold material properties including friction and restitution.
//

// Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape
func CreateCircleShape(bodyId BodyId, def *ShapeDef, circle Circle) ShapeId {
	cbody := *cast[C.b2BodyId](&bodyId)
	cdef := cast[C.b2ShapeDef](def)
	ccircle := cast[C.b2Circle](&circle)
	r := C.b2CreateCircleShape(cbody, cdef, ccircle)
	return *cast[ShapeId](&r)
}

// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape
func CreateSegmentShape(bodyId BodyId, def *ShapeDef, segment Segment) ShapeId {
	cbody := *cast[C.b2BodyId](&bodyId)
	cdef := cast[C.b2ShapeDef](def)
	csegment := cast[C.b2Segment](&segment)
	r := C.b2CreateSegmentShape(cbody, cdef, csegment)
	return *cast[ShapeId](&r)
}

// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape, this will be b2_nullShapeId if the length is too small.
func CreateCapsuleShape(bodyId BodyId, def *ShapeDef, capsule Capsule) ShapeId {
	cbody := *cast[C.b2BodyId](&bodyId)
	cdef := cast[C.b2ShapeDef](def)
	ccapsule := cast[C.b2Capsule](&capsule)
	r := C.b2CreateCapsuleShape(cbody, cdef, ccapsule)
	return *cast[ShapeId](&r)
}

// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape
func CreatePolygonShape(bodyId BodyId, def *ShapeDef, polygon Polygon) ShapeId {
	cbody := *cast[C.b2BodyId](&bodyId)
	cdef := cast[C.b2ShapeDef](def)
	cpolygon := cast[C.b2Polygon](&polygon)
	r := C.b2CreatePolygonShape(cbody, cdef, cpolygon)
	return *cast[ShapeId](&r)
}

// Destroy a shape. You may defer the body mass update which can improve performance if several shapes on a
//	body are destroyed at once.
//	@see b2Body_ApplyMassFromShapes
func DestroyShape(shapeId ShapeId, updateBodyMass bool) {
	C.b2DestroyShape(*cast[C.b2ShapeId](&shapeId), C.bool(updateBodyMass))
}

// Shape identifier validation. Provides validation for up to 64K allocations.
func Shape_IsValid(id ShapeId) bool {
	return bool(C.b2Shape_IsValid(*cast[C.b2ShapeId](&id)))
}

// Get the type of a shape
func Shape_GetType(shapeId ShapeId) ShapeType {
	return ShapeType(C.b2Shape_GetType(*cast[C.b2ShapeId](&shapeId)))
}

// Get the id of the body that a shape is attached to
func Shape_GetBody(shapeId ShapeId) BodyId {
	r := C.b2Shape_GetBody(*cast[C.b2ShapeId](&shapeId))
	return *cast[BodyId](&r)
}

// Get the world that owns this shape
func Shape_GetWorld(shapeId ShapeId) WorldId {
	r := C.b2Shape_GetWorld(*cast[C.b2ShapeId](&shapeId))
	return *cast[WorldId](&r)
}

// Returns true if the shape is a sensor. It is not possible to change a shape
// from sensor to solid dynamically because this breaks the contract for
// sensor events.
func Shape_IsSensor(shapeId ShapeId) bool {
	return bool(C.b2Shape_IsSensor(*cast[C.b2ShapeId](&shapeId)))
}

/*
// Set the user data for a shape
B2_API void b2Shape_SetUserData( ShapeId shapeId, void* userData );

// Get the user data for a shape. This is useful when you get a shape id
// from an event or query.
B2_API void* b2Shape_GetUserData( ShapeId shapeId );
*/
// Set the mass density of a shape, usually in kg/m^2.
// This will optionally update the mass properties on the parent body.
// @see b2ShapeDef::density, b2Body_ApplyMassFromShapes
func Shape_SetDensity(shapeId ShapeId, density float32, updateBodyMass bool) {
	C.b2Shape_SetDensity(*cast[C.b2ShapeId](&shapeId), C.float(density), C.bool(updateBodyMass))
}

// Get the density of a shape, usually in kg/m^2
func Shape_GetDensity(shapeId ShapeId) float32 {
	return float32(C.b2Shape_GetDensity(*cast[C.b2ShapeId](&shapeId)))
}

// Set the friction on a shape
func Shape_SetFriction(shapeId ShapeId, friction float32) {
	C.b2Shape_SetFriction(*cast[C.b2ShapeId](&shapeId), C.float(friction))
}

// Get the friction of a shape
func Shape_GetFriction(shapeId ShapeId) float32 {
	return float32(C.b2Shape_GetFriction(*cast[C.b2ShapeId](&shapeId)))
}

// Set the shape restitution (bounciness)
func Shape_SetRestitution(shapeId ShapeId, restitution float32) {
	C.b2Shape_SetRestitution(*cast[C.b2ShapeId](&shapeId), C.float(restitution))
}

// Get the shape restitution
func Shape_GetRestitution(shapeId ShapeId) float32 {
	return float32(C.b2Shape_GetRestitution(*cast[C.b2ShapeId](&shapeId)))
}

// Set the user material identifier
func Shape_SetUserMaterial(shapeId ShapeId, material uint64) {
	C.b2Shape_SetUserMaterial(*cast[C.b2ShapeId](&shapeId), C.uint64_t(material))
}

// Get the user material identifier
func Shape_GetUserMaterial(shapeId ShapeId) uint64 {
	return uint64(C.b2Shape_GetUserMaterial(*cast[C.b2ShapeId](&shapeId)))
}

// Set the shape surface material
func Shape_SetSurfaceMaterial(shapeId ShapeId, surfaceMaterial *SurfaceMaterial) {
	C.b2Shape_SetSurfaceMaterial(*cast[C.b2ShapeId](&shapeId), cast[C.b2SurfaceMaterial](surfaceMaterial))
}

// Get the shape surface material
func Shape_GetSurfaceMaterial(shapeId ShapeId) SurfaceMaterial {
	r := C.b2Shape_GetSurfaceMaterial(*cast[C.b2ShapeId](&shapeId))
	return *cast[SurfaceMaterial](&r)
}

// Get the shape filter
func Shape_GetFilter(shapeId ShapeId) Filter {
	r := C.b2Shape_GetFilter(*cast[C.b2ShapeId](&shapeId))
	return *cast[Filter](&r)
}

// Set the current filter. This is almost as expensive as recreating the shape. This may cause
// contacts to be immediately destroyed. However contacts are not created until the next world step.
// Sensor overlap state is also not updated until the next world step.
// @see b2ShapeDef::filter
func Shape_SetFilter(shapeId ShapeId, filter Filter) {
	C.b2Shape_SetFilter(*cast[C.b2ShapeId](&shapeId), *cast[C.b2Filter](&filter))
}

// Enable sensor events for this shape.
// @see b2ShapeDef::enableSensorEvents
func Shape_EnableSensorEvents(shapeId ShapeId, flag bool) {
	C.b2Shape_EnableSensorEvents(*cast[C.b2ShapeId](&shapeId), C.bool(flag))
}

// Returns true if sensor events are enabled.
func Shape_AreSensorEventsEnabled(shapeId ShapeId) bool {
	return bool(C.b2Shape_AreSensorEventsEnabled(*cast[C.b2ShapeId](&shapeId)))
}

// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
// @see b2ShapeDef::enableContactEvents
// @warning changing this at run-time may lead to lost begin/end events
func Shape_EnableContactEvents(shapeId ShapeId, flag bool) {
	C.b2Shape_EnableContactEvents(*cast[C.b2ShapeId](&shapeId), C.bool(flag))
}

// Returns true if contact events are enabled
func Shape_AreContactEventsEnabled(shapeId ShapeId) bool {
	return bool(C.b2Shape_AreContactEventsEnabled(*cast[C.b2ShapeId](&shapeId)))
}

// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
// and must be carefully handled due to multithreading. Ignored for sensors.
// @see b2PreSolveFcn
func Shape_EnablePreSolveEvents(shapeId ShapeId, flag bool) {
	C.b2Shape_EnablePreSolveEvents(*cast[C.b2ShapeId](&shapeId), C.bool(flag))
}

// Returns true if pre-solve events are enabled
func Shape_ArePreSolveEventsEnabled(shapeId ShapeId) bool {
	return bool(C.b2Shape_ArePreSolveEventsEnabled(*cast[C.b2ShapeId](&shapeId)))
}

// Enable contact hit events for this shape. Ignored for sensors.
// @see b2WorldDef.hitEventThreshold
func Shape_EnableHitEvents(shapeId ShapeId, flag bool) {
	C.b2Shape_EnableHitEvents(*cast[C.b2ShapeId](&shapeId), C.bool(flag))
}

// Returns true if hit events are enabled
func Shape_AreHitEventsEnabled(shapeId ShapeId) bool {
	return bool(C.b2Shape_AreHitEventsEnabled(*cast[C.b2ShapeId](&shapeId)))
}

// Test a point for overlap with a shape
func Shape_TestPoint(shapeId ShapeId, point Vec2) bool {
	return bool(C.b2Shape_TestPoint(*cast[C.b2ShapeId](&shapeId), *cast[C.b2Vec2](&point)))
}

// Ray cast a shape directly
func Shape_RayCast(shapeId ShapeId, input RayCastInput) CastOutput {
	r := C.b2Shape_RayCast(*cast[C.b2ShapeId](&shapeId), cast[C.b2RayCastInput](&input))
	return *cast[CastOutput](&r)
}

// Get a copy of the shape's circle. Asserts the type is correct.
func Shape_GetCircle(shapeId ShapeId) Circle {
	r := C.b2Shape_GetCircle(*cast[C.b2ShapeId](&shapeId))
	return *cast[Circle](&r)
}

// Get a copy of the shape's line segment. Asserts the type is correct.
func Shape_GetSegment(shapeId ShapeId) Segment {
	r := C.b2Shape_GetSegment(*cast[C.b2ShapeId](&shapeId))
	return *cast[Segment](&r)
}

// Get a copy of the shape's chain segment. These come from chain shapes.
// Asserts the type is correct.
func Shape_GetChainSegment(shapeId ShapeId) ChainSegment {
	r := C.b2Shape_GetChainSegment(*cast[C.b2ShapeId](&shapeId))
	return *cast[ChainSegment](&r)
}

// Get a copy of the shape's capsule. Asserts the type is correct.
func Shape_GetCapsule(shapeId ShapeId) Capsule {
	r := C.b2Shape_GetCapsule(*cast[C.b2ShapeId](&shapeId))
	return *cast[Capsule](&r)
}

// Get a copy of the shape's convex polygon. Asserts the type is correct.
func Shape_GetPolygon(shapeId ShapeId) Polygon {
	r := C.b2Shape_GetPolygon(*cast[C.b2ShapeId](&shapeId))
	return *cast[Polygon](&r)
}

// Allows you to change a shape to be a circle or update the current circle.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func Shape_SetCircle(shapeId ShapeId, circle Circle) {
	C.b2Shape_SetCircle(*cast[C.b2ShapeId](&shapeId), cast[C.b2Circle](&circle))
}

// Allows you to change a shape to be a capsule or update the current capsule.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func Shape_SetCapsule(shapeId ShapeId, capsule Capsule) {
	C.b2Shape_SetCapsule(*cast[C.b2ShapeId](&shapeId), cast[C.b2Capsule](&capsule))
}

// Allows you to change a shape to be a segment or update the current segment.
func Shape_SetSegment(shapeId ShapeId, segment Segment) {
	C.b2Shape_SetSegment(*cast[C.b2ShapeId](&shapeId), cast[C.b2Segment](&segment))
}

// Allows you to change a shape to be a polygon or update the current polygon.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func Shape_SetPolygon(shapeId ShapeId, polygon Polygon) {
	C.b2Shape_SetPolygon(*cast[C.b2ShapeId](&shapeId), cast[C.b2Polygon](&polygon))
}

// Get the parent chain id if the shape type is a chain segment, otherwise
// returns b2_nullChainId.
func Shape_GetParentChain(shapeId ShapeId) ChainId {
	r := C.b2Shape_GetParentChain(*cast[C.b2ShapeId](&shapeId))
	return *cast[ChainId](&r)
}

// Get the maximum capacity required for retrieving all the touching contacts on a shape
func Shape_GetContactCapacity(shapeId ShapeId) int {
	return int(C.b2Shape_GetContactCapacity(*cast[C.b2ShapeId](&shapeId)))
}

// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
// @note Box2D uses speculative collision so some contact points may be separated.
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
func Shape_GetContactData(shapeId ShapeId, contactData *ContactData, capacity int) int {
	return int(C.b2Shape_GetContactData(*cast[C.b2ShapeId](&shapeId), cast[C.b2ContactData](contactData), C.int(capacity)))
}

// Get the maximum capacity required for retrieving all the overlapped shapes on a sensor shape.
// This returns 0 if the provided shape is not a sensor.
// @param shapeId the id of a sensor shape
// @returns the required capacity to get all the overlaps in b2Shape_GetSensorData
func Shape_GetSensorCapacity(shapeId ShapeId) int {
	return int(C.b2Shape_GetSensorCapacity(*cast[C.b2ShapeId](&shapeId)))
}

// Get the overlap data for a sensor shape computed the previous world step.
// @param shapeId the id of a sensor shape
// @param visitorIds a user allocated array that is filled with the overlapping shapes (visitors)
// @param capacity the capacity of overlappedShapes
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
// @warning overlaps may contain destroyed shapes so use b2Shape_IsValid to confirm each overlap
func Shape_GetSensorData(shapeId ShapeId, visitorIds *ShapeId, capacity int) int {
	return int(C.b2Shape_GetSensorData(*cast[C.b2ShapeId](&shapeId), cast[C.b2ShapeId](visitorIds), C.int(capacity)))
}

// Get the current world AABB
func Shape_GetAABB(shapeId ShapeId) AABB {
	r := C.b2Shape_GetAABB(*cast[C.b2ShapeId](&shapeId))
	return *cast[AABB](&r)
}

// Compute the mass data for a shape
func Shape_ComputeMassData(shapeId ShapeId) MassData {
	r := C.b2Shape_ComputeMassData(*cast[C.b2ShapeId](&shapeId))
	return *cast[MassData](&r)
}

// Get the closest point on a shape to a target point. Target and result are in world space.
// todo need sample
func Shape_GetClosestPoint(shapeId ShapeId, target Vec2) Vec2 {
	r := C.b2Shape_GetClosestPoint(*cast[C.b2ShapeId](&shapeId), *cast[C.b2Vec2](&target))
	return *cast[Vec2](&r)
}

// Apply a wind force to the body for this shape using the density of air. This considers
// the projected area of the shape in the wind direction. This also considers
// the relative velocity of the shape.
// @param shapeId the shape id
// @param wind the wind velocity in world space
// @param drag the drag coefficient, the force that opposes the relative velocity
// @param lift the lift coefficient, the force that is perpendicular to the relative velocity
// @param wake should this wake the body
func Shape_ApplyWind(shapeId ShapeId, wind Vec2, drag float32, lift float32, wake bool) {
	C.b2Shape_ApplyWind(*cast[C.b2ShapeId](&shapeId), *cast[C.b2Vec2](&wind), C.float(drag), C.float(lift), C.bool(wake))
}

/*
// Chain Shape

// Create a chain shape
// @see b2ChainDef for details
B2_API b2ChainId b2CreateChain( b2BodyId bodyId, const b2ChainDef* def );

// Destroy a chain shape
B2_API void b2DestroyChain( b2ChainId chainId );

// Get the world that owns this chain shape
B2_API b2WorldId b2Chain_GetWorld( b2ChainId chainId );

// Get the number of segments on this chain
B2_API int b2Chain_GetSegmentCount( b2ChainId chainId );

// Fill a user array with chain segment shape ids up to the specified capacity. Returns
// the actual number of segments returned.
B2_API int b2Chain_GetSegments( b2ChainId chainId, ShapeId* segmentArray, int capacity );

// Get the number of materials used on this chain. Must be 1 or the number of segments.
B2_API int b2Chain_GetSurfaceMaterialCount( b2ChainId chainId );

// Set a chain material. If the chain has only one material, this material is applied to all
// segments. Otherwise it is applied to a single segment.
B2_API void b2Chain_SetSurfaceMaterial( b2ChainId chainId, const SurfaceMaterial* material, int materialIndex );

// Get a chain material by index.
B2_API SurfaceMaterial b2Chain_GetSurfaceMaterial( b2ChainId chainId, int materialIndex );

// Chain identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Chain_IsValid( b2ChainId id );

//
// @defgroup joint Joint
// @brief Joints allow you to connect rigid bodies together while allowing various forms of relative motions.
//

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
//
// @defgroup contact Contact
// Access to contacts
//

// Contact identifier validation. Provides validation for up to 2^32 allocations.
func Contact_IsValid(id ContactId) bool {
	return bool(C.b2Contact_IsValid(*cast[C.b2ContactId](&id)))
}

// Get the data for a contact. The manifold may have no points if the contact is not touching.
func Contact_GetData(contactId ContactId) ContactData {
	r := C.b2Contact_GetData(*cast[C.b2ContactId](&contactId))
	return *cast[ContactData](&r)
}
