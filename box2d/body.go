package box2d

/*
#include "box2d/box2d.h"
*/
import "C"

// Body id references a body instance. This should be treated as an opaque handle.
type BodyId struct {
	index1     int32
	world0     uint16
	generation uint16
}

func (b BodyId) Defer() {
	b.Destroy()
}

// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
// Do not keep references to the associated shapes and joints.
func (b BodyId) Destroy() {
	C.b2DestroyBody(*cast[C.b2BodyId](&b))
}

// Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape
func (b BodyId) CreateCircleShape(def *ShapeDef, circle Circle) ShapeId {
	cbody := *cast[C.b2BodyId](&b)
	cdef := cast[C.b2ShapeDef](def)
	ccircle := cast[C.b2Circle](&circle)
	r := C.b2CreateCircleShape(cbody, cdef, ccircle)
	return *cast[ShapeId](&r)
}

// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape
func (b BodyId) CreateSegmentShape(def *ShapeDef, segment Segment) ShapeId {
	cbody := *cast[C.b2BodyId](&b)
	cdef := cast[C.b2ShapeDef](def)
	csegment := cast[C.b2Segment](&segment)
	r := C.b2CreateSegmentShape(cbody, cdef, csegment)
	return *cast[ShapeId](&r)
}

// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape, this will be b2_nullShapeId if the length is too small.
func (b BodyId) CreateCapsuleShape(def *ShapeDef, capsule Capsule) ShapeId {
	cbody := *cast[C.b2BodyId](&b)
	cdef := cast[C.b2ShapeDef](def)
	ccapsule := cast[C.b2Capsule](&capsule)
	r := C.b2CreateCapsuleShape(cbody, cdef, ccapsule)
	return *cast[ShapeId](&r)
}

// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape
func (b BodyId) CreatePolygonShape(def *ShapeDef, polygon Polygon) ShapeId {
	cbody := *cast[C.b2BodyId](&b)
	cdef := cast[C.b2ShapeDef](def)
	cpolygon := cast[C.b2Polygon](&polygon)
	r := C.b2CreatePolygonShape(cbody, cdef, cpolygon)
	return *cast[ShapeId](&r)
}

// Body identifier validation. A valid body exists in a world and is non-null.
// This can be used to detect orphaned ids. Provides validation for up to 64K allocations.
func (b BodyId) IsValid() bool {
	return bool(C.b2Body_IsValid(*cast[C.b2BodyId](&b)))
}

// Get the body type: static, kinematic, or dynamic
func (b BodyId) GetType() BodyType {
	return BodyType(C.b2Body_GetType(*cast[C.b2BodyId](&b)))
}

// Change the body type. This is an expensive operation. This automatically updates the mass
// properties regardless of the automatic mass setting.
func (b BodyId) SetType(_type BodyType) {
	C.b2Body_SetType(*cast[C.b2BodyId](&b), C.b2BodyType(_type))
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
func (b BodyId) GetPosition() Vec2 {
	r := C.b2Body_GetPosition(*cast[C.b2BodyId](&b))
	return *cast[Vec2](&r)
}

// Get the world rotation of a body as a cosine/sine pair (complex number)
func (b BodyId) GetRotation() Rot {
	r := C.b2Body_GetRotation(*cast[C.b2BodyId](&b))
	return *cast[Rot](&r)
}

// Get the world transform of a body.
func (b BodyId) GetTransform() Transform {
	r := C.b2Body_GetTransform(*cast[C.b2BodyId](&b))
	return *cast[Transform](&r)
}

// Set the world transform of a body. This acts as a teleport and is fairly expensive.
// @note Generally you should create a body with then intended transform.
// @see b2BodyDef::position and b2BodyDef::rotation
func (b BodyId) SetTransform(position Vec2, rotation Rot) {
	C.b2Body_SetTransform(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&position), *cast[C.b2Rot](&rotation))
}

// Get a local point on a body given a world point
func (b BodyId) GetLocalPoint(worldPoint Vec2) Vec2 {
	r := C.b2Body_GetLocalPoint(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&worldPoint))
	return *cast[Vec2](&r)
}

// Get a world point on a body given a local point
func (b BodyId) GetWorldPoint(localPoint Vec2) Vec2 {
	r := C.b2Body_GetWorldPoint(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&localPoint))
	return *cast[Vec2](&r)
}

// Get a local vector on a body given a world vector
func (b BodyId) GetLocalVector(worldVector Vec2) Vec2 {
	r := C.b2Body_GetLocalVector(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&worldVector))
	return *cast[Vec2](&r)
}

// Get a world vector on a body given a local vector
func (b BodyId) GetWorldVector(localVector Vec2) Vec2 {
	r := C.b2Body_GetWorldVector(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&localVector))
	return *cast[Vec2](&r)
}

// Get the linear velocity of a body's center of mass. Usually in meters per second.
func (b BodyId) GetLinearVelocity() Vec2 {
	r := C.b2Body_GetLinearVelocity(*cast[C.b2BodyId](&b))
	return *cast[Vec2](&r)
}

// Get the angular velocity of a body in radians per second
func (b BodyId) GetAngularVelocity() float32 {
	return float32(C.b2Body_GetAngularVelocity(*cast[C.b2BodyId](&b)))
}

// Set the linear velocity of a body. Usually in meters per second.
func (b BodyId) SetLinearVelocity(linearVelocity Vec2) {
	C.b2Body_SetLinearVelocity(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&linearVelocity))
}

// Set the angular velocity of a body in radians per second
func (b BodyId) SetAngularVelocity(angularVelocity float32) {
	C.b2Body_SetAngularVelocity(*cast[C.b2BodyId](&b), C.float(angularVelocity))
}

// Set the velocity to reach the given transform after a given time step.
// The result will be close but maybe not exact. This is meant for kinematic bodies.
// The target is not applied if the velocity would be below the sleep threshold and
// the body is currently asleep.
// @param bodyId The body id
// @param target The target transform for the body
// @param timeStep The time step of the next call to b2World_Step
// @param wake Option to wake the body or not
func (b BodyId) SetTargetTransform(target Transform, timeStep float32, wake bool) {
	C.b2Body_SetTargetTransform(*cast[C.b2BodyId](&b), *cast[C.b2Transform](&target), C.float(timeStep), C.bool(wake))
}

// Get the linear velocity of a local point attached to a body. Usually in meters per second.
func (b BodyId) GetLocalPointVelocity(localPoint Vec2) Vec2 {
	r := C.b2Body_GetLocalPointVelocity(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&localPoint))
	return *cast[Vec2](&r)
}

// Get the linear velocity of a world point attached to a body. Usually in meters per second.
func (b BodyId) GetWorldPointVelocity(worldPoint Vec2) Vec2 {
	r := C.b2Body_GetWorldPointVelocity(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&worldPoint))
	return *cast[Vec2](&r)
}

// Apply a force at a world point. If the force is not applied at the center of mass,
// it will generate a torque and affect the angular velocity. This optionally wakes up the body.
// The force is ignored if the body is not awake.
// @param bodyId The body id
// @param force The world force vector, usually in newtons (N)
// @param point The world position of the point of application
// @param wake Option to wake up the body
func (b BodyId) ApplyForce(force Vec2, point Vec2, wake bool) {
	C.b2Body_ApplyForce(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&force), *cast[C.b2Vec2](&point), C.bool(wake))
}

// Apply a force to the center of mass. This optionally wakes up the body.
// The force is ignored if the body is not awake.
// @param bodyId The body id
// @param force the world force vector, usually in newtons (N).
// @param wake also wake up the body
func (b BodyId) ApplyForceToCenter(force Vec2, wake bool) {
	C.b2Body_ApplyForceToCenter(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&force), C.bool(wake))
}

// Apply a torque. This affects the angular velocity without affecting the linear velocity.
// This optionally wakes the body. The torque is ignored if the body is not awake.
// @param bodyId The body id
// @param torque about the z-axis (out of the screen), usually in N*m.
// @param wake also wake up the body
func (b BodyId) ApplyTorque(torque float32, wake bool) {
	C.b2Body_ApplyTorque(*cast[C.b2BodyId](&b), C.float(torque), C.bool(wake))
}

// Clear the force and torque on this body. Forces and torques are automatically cleared after each world
// step. So this only needs to be called if the application wants to remove the effect of previous
// calls to apply forces and torques before the world step is called.
// @param bodyId The body id
func (b BodyId) ClearForces() {
	C.b2Body_ClearForces(*cast[C.b2BodyId](&b))
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
func (b BodyId) ApplyLinearImpulse(impulse Vec2, point Vec2, wake bool) {
	C.b2Body_ApplyLinearImpulse(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&impulse), *cast[C.b2Vec2](&point), C.bool(wake))
}

// Apply an impulse to the center of mass. This immediately modifies the velocity.
// The impulse is ignored if the body is not awake. This optionally wakes the body.
// @param bodyId The body id
// @param impulse the world impulse vector, usually in N*s or kg*m/s.
// @param wake also wake up the body
// @warning This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
func (b BodyId) ApplyLinearImpulseToCenter(impulse Vec2, wake bool) {
	C.b2Body_ApplyLinearImpulseToCenter(*cast[C.b2BodyId](&b), *cast[C.b2Vec2](&impulse), C.bool(wake))
}

// Apply an angular impulse. The impulse is ignored if the body is not awake.
// This optionally wakes the body.
// @param bodyId The body id
// @param impulse the angular impulse, usually in units of kg*m*m/s
// @param wake also wake up the body
// @warning This should be used for one-shot impulses. If you need a steady torque,
// use a torque instead, which will work better with the sub-stepping solver.
func (b BodyId) ApplyAngularImpulse(impulse float32, wake bool) {
	C.b2Body_ApplyAngularImpulse(*cast[C.b2BodyId](&b), C.float(impulse), C.bool(wake))
}

// Get the mass of the body, usually in kilograms
func (b BodyId) GetMass() float32 {
	return float32(C.b2Body_GetMass(*cast[C.b2BodyId](&b)))
}

// Get the rotational inertia of the body, usually in kg*m^2
func (b BodyId) GetRotationalInertia() float32 {
	return float32(C.b2Body_GetRotationalInertia(*cast[C.b2BodyId](&b)))
}

// Get the center of mass position of the body in local space
func (b BodyId) GetLocalCenterOfMass() Vec2 {
	r := C.b2Body_GetLocalCenterOfMass(*cast[C.b2BodyId](&b))
	return *cast[Vec2](&r)
}

// Get the center of mass position of the body in world space
func (b BodyId) GetWorldCenterOfMass() Vec2 {
	r := C.b2Body_GetWorldCenterOfMass(*cast[C.b2BodyId](&b))
	return *cast[Vec2](&r)
}

// Override the body's mass properties. Normally this is computed automatically using the
// shape geometry and density. This information is lost if a shape is added or removed or if the
// body type changes.
func (b BodyId) SetMassData(massData MassData) {
	C.b2Body_SetMassData(*cast[C.b2BodyId](&b), *cast[C.b2MassData](&massData))
}

// Get the mass data for a body
func (b BodyId) GetMassData() MassData {
	r := C.b2Body_GetMassData(*cast[C.b2BodyId](&b))
	return *cast[MassData](&r)
}

// This updates the mass properties to the sum of the mass properties of the shapes.
// This normally does not need to be called unless you called SetMassData to override
// the mass and you later want to reset the mass.
// You may also use this when automatic mass computation has been disabled.
// You should call this regardless of body type.
// Note that sensor shapes may have mass.
func (b BodyId) ApplyMassFromShapes() {
	C.b2Body_ApplyMassFromShapes(*cast[C.b2BodyId](&b))
}

// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
func (b BodyId) SetLinearDamping(linearDamping float32) {
	C.b2Body_SetLinearDamping(*cast[C.b2BodyId](&b), C.float(linearDamping))
}

// Get the current linear damping.
func (b BodyId) GetLinearDamping() float32 {
	return float32(C.b2Body_GetLinearDamping(*cast[C.b2BodyId](&b)))
}

// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
func (b BodyId) SetAngularDamping(angularDamping float32) {
	C.b2Body_SetAngularDamping(*cast[C.b2BodyId](&b), C.float(angularDamping))
}

// Get the current angular damping.
func (b BodyId) GetAngularDamping() float32 {
	return float32(C.b2Body_GetAngularDamping(*cast[C.b2BodyId](&b)))
}

// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
// @see b2BodyDef::gravityScale
func (b BodyId) SetGravityScale(gravityScale float32) {
	C.b2Body_SetGravityScale(*cast[C.b2BodyId](&b), C.float(gravityScale))
}

// Get the current gravity scale
func (b BodyId) GetGravityScale() float32 {
	return float32(C.b2Body_GetGravityScale(*cast[C.b2BodyId](&b)))
}

// @return true if this body is awake
func (b BodyId) IsAwake() bool {
	return bool(C.b2Body_IsAwake(*cast[C.b2BodyId](&b)))
}

// Wake a body from sleep. This wakes the entire island the body is touching.
// @warning Putting a body to sleep will put the entire island of bodies touching this body to sleep,
// which can be expensive and possibly unintuitive.
func (b BodyId) SetAwake(awake bool) {
	C.b2Body_SetAwake(*cast[C.b2BodyId](&b), C.bool(awake))
}

// Wake bodies touching this body. Works for static bodies.
func (b BodyId) WakeTouching() {
	C.b2Body_WakeTouching(*cast[C.b2BodyId](&b))
}

// Enable or disable sleeping for this body. If sleeping is disabled the body will wake.
func (b BodyId) EnableSleep(enableSleep bool) {
	C.b2Body_EnableSleep(*cast[C.b2BodyId](&b), C.bool(enableSleep))
}

// Returns true if sleeping is enabled for this body
func (b BodyId) IsSleepEnabled() bool {
	return bool(C.b2Body_IsSleepEnabled(*cast[C.b2BodyId](&b)))
}

// Set the sleep threshold, usually in meters per second
func (b BodyId) SetSleepThreshold(sleepThreshold float32) {
	C.b2Body_SetSleepThreshold(*cast[C.b2BodyId](&b), C.float(sleepThreshold))
}

// Get the sleep threshold, usually in meters per second.
func (b BodyId) GetSleepThreshold() float32 {
	return float32(C.b2Body_GetSleepThreshold(*cast[C.b2BodyId](&b)))
}

// Returns true if this body is enabled
func (b BodyId) IsEnabled() bool {
	return bool(C.b2Body_IsEnabled(*cast[C.b2BodyId](&b)))
}

// Disable a body by removing it completely from the simulation. This is expensive.
func (b BodyId) Disable() {
	C.b2Body_Disable(*cast[C.b2BodyId](&b))
}

// Enable a body by adding it to the simulation. This is expensive.
func (b BodyId) Enable() {
	C.b2Body_Enable(*cast[C.b2BodyId](&b))
}

// Set the motion locks on this body.
func (b BodyId) SetMotionLocks(locks MotionLocks) {
	C.b2Body_SetMotionLocks(*cast[C.b2BodyId](&b), *cast[C.b2MotionLocks](&locks))
}

// Get the motion locks for this body.
func (b BodyId) GetMotionLocks() MotionLocks {
	r := C.b2Body_GetMotionLocks(*cast[C.b2BodyId](&b))
	return *cast[MotionLocks](&r)
}

// Set this body to be a bullet. A bullet does continuous collision detection
// against dynamic bodies (but not other bullets).
func (b BodyId) SetBullet(flag bool) {
	C.b2Body_SetBullet(*cast[C.b2BodyId](&b), C.bool(flag))
}

// Is this body a bullet?
func (b BodyId) IsBullet() bool {
	return bool(C.b2Body_IsBullet(*cast[C.b2BodyId](&b)))
}

// Enable/disable contact events on all shapes.
// @see b2ShapeDef::enableContactEvents
// @warning changing this at runtime may cause mismatched begin/end touch events
func (b BodyId) EnableContactEvents(flag bool) {
	C.b2Body_EnableContactEvents(*cast[C.b2BodyId](&b), C.bool(flag))
}

// Enable/disable hit events on all shapes
// @see b2ShapeDef::enableHitEvents
func (b BodyId) EnableHitEvents(flag bool) {
	C.b2Body_EnableHitEvents(*cast[C.b2BodyId](&b), C.bool(flag))
}

// Get the world that owns this body
func (b BodyId) GetWorld() WorldId {
	r := C.b2Body_GetWorld(*cast[C.b2BodyId](&b))
	return *cast[WorldId](&r)
}

// Get the number of shapes on this body
func (b BodyId) GetShapeCount() int {
	return int(C.b2Body_GetShapeCount(*cast[C.b2BodyId](&b)))
}

// Get the shape ids for all shapes on this body, up to the provided capacity.
// @returns the number of shape ids stored in the user array
func (b BodyId) GetShapes(shapeArray *ShapeId, capacity int) int {
	return int(C.b2Body_GetShapes(*cast[C.b2BodyId](&b), cast[C.b2ShapeId](shapeArray), C.int(capacity)))
}

// Get the number of joints on this body
func (b BodyId) GetJointCount() int {
	return int(C.b2Body_GetJointCount(*cast[C.b2BodyId](&b)))
}

// Get the joint ids for all joints on this body, up to the provided capacity
// @returns the number of joint ids stored in the user array
func (b BodyId) GetJoints(jointArray *JointId, capacity int) int {
	return int(C.b2Body_GetJoints(*cast[C.b2BodyId](&b), cast[C.b2JointId](jointArray), C.int(capacity)))
}

// Get the maximum capacity required for retrieving all the touching contacts on a body
func (b BodyId) GetContactCapacity() int {
	return int(C.b2Body_GetContactCapacity(*cast[C.b2BodyId](&b)))
}

// Get the touching contact data for a body.
// @note Box2D uses speculative collision so some contact points may be separated.
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
func (b BodyId) GetContactData(contactData *ContactData, capacity int) int {
	return int(C.b2Body_GetContactData(*cast[C.b2BodyId](&b), cast[C.b2ContactData](contactData), C.int(capacity)))
}

// Get the current world AABB that contains all the attached shapes. Note that this may not encompass the body origin.
// If there are no shapes attached then the returned AABB is empty and centered on the body origin.
func (b BodyId) ComputeAABB() AABB {
	r := C.b2Body_ComputeAABB(*cast[C.b2BodyId](&b))
	return *cast[AABB](&r)
}
