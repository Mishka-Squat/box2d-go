package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

// World id references a world instance. This should be treated as an opaque handle.
type WorldId struct {
	index1     uint16
	generation uint16
}

func (w WorldId) Destroy() {
	DestroyWorld(w)
}

func (w WorldId) Defer() {
	w.Destroy()
}

func (w WorldId) IsValid() bool {
	return World_IsValid(w)
}

// Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
// on the stack and pass it as a pointer.
// @code{.c}
// b2BodyDef bodyDef = b2DefaultBodyDef();
// b2BodyId myBodyId = b2CreateBody(myWorldId, &bodyDef);
// @endcode
// @warning This function is locked during callbacks.
func (w WorldId) CreateBody(def *BodyDef) BodyId {
	return CreateBody(w, def)
}

func (w WorldId) Step(timeStep float32, subStepCount int) {
	World_Step(w, timeStep, subStepCount)
}

// Call this to draw shapes and other debug draw data
func (w WorldId) Draw(draw *DebugDraw) {
	World_Draw(w, draw)
}

// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
func (w WorldId) GetBodyEvents() BodyEvents {
	return World_GetBodyEvents(w)
}

// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
func (w WorldId) GetSensorEvents() SensorEvents {
	return World_GetSensorEvents(w)
}

// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
func (w WorldId) GetContactEvents() ContactEvents {
	return World_GetContactEvents(w)
}

// Get the joint events for the current time step. The event data is transient. Do not store a reference to this data.
func (w WorldId) GetJointEvents() JointEvents {
	return World_GetJointEvents(w)
}

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
	DestroyBody(b)
}

func (b BodyId) CreateCircleShape(def *ShapeDef, circle *Circle) ShapeId {
	return CreateCircleShape(b, def, circle)
}

// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape
func (b BodyId) CreateSegmentShape(def *ShapeDef, segment *Segment) ShapeId {
	return CreateSegmentShape(b, def, segment)
}

// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape, this will be b2_nullShapeId if the length is too small.
func (b BodyId) CreateCapsuleShape(def *ShapeDef, capsule *Capsule) ShapeId {
	return CreateCapsuleShape(b, def, capsule)
}

// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
// @return the shape id for accessing the shape
func (b BodyId) CreatePolygonShape(def *ShapeDef, polygon *Polygon) ShapeId {
	return CreatePolygonShape(b, def, polygon)
}

// Body identifier validation. A valid body exists in a world and is non-null.
// This can be used to detect orphaned ids. Provides validation for up to 64K allocations.
func (b BodyId) IsValid() bool {
	return Body_IsValid(b)
}

// Get the body type: static, kinematic, or dynamic
func (b BodyId) GetType() BodyType {
	return Body_GetType(b)
}

// Change the body type. This is an expensive operation. This automatically updates the mass
// properties regardless of the automatic mass setting.
func (b BodyId) SetType(_type BodyType) {
	Body_SetType(b, _type)
}

// Get the world position of a body. This is the location of the body origin.
func (b BodyId) GetPosition() Vec2 {
	return Body_GetPosition(b)
}

// Get the world rotation of a body as a cosine/sine pair (complex number)
func (b BodyId) GetRotation() Rot {
	return Body_GetRotation(b)
}

// Get the world transform of a body.
func (b BodyId) GetTransform() Transform {
	return Body_GetTransform(b)
}

// Set the world transform of a body. This acts as a teleport and is fairly expensive.
// @note Generally you should create a body with then intended transform.
// @see b2BodyDef::position and b2BodyDef::rotation
func (b BodyId) SetTransform(position Vec2, rotation Rot) {
	Body_SetTransform(b, position, rotation)
}

// Get a local point on a body given a world point
func (b BodyId) GetLocalPoint(worldPoint Vec2) Vec2 {
	return Body_GetLocalPoint(b, worldPoint)
}

// Get a world point on a body given a local point
func (b BodyId) GetWorldPoint(localPoint Vec2) Vec2 {
	return Body_GetWorldPoint(b, localPoint)
}

// Get a local vector on a body given a world vector
func (b BodyId) GetLocalVector(worldVector Vec2) Vec2 {
	return Body_GetLocalVector(b, worldVector)
}

// Get a world vector on a body given a local vector
func (b BodyId) GetWorldVector(localVector Vec2) Vec2 {
	return Body_GetWorldVector(b, localVector)
}

// Get the linear velocity of a body's center of mass. Usually in meters per second.
func (b BodyId) GetLinearVelocity() Vec2 {
	return Body_GetLinearVelocity(b)
}

// Get the angular velocity of a body in radians per second
func (b BodyId) GetAngularVelocity() float32 {
	return Body_GetAngularVelocity(b)
}

// Set the linear velocity of a body. Usually in meters per second.
func (b BodyId) SetLinearVelocity(linearVelocity Vec2) {
	Body_SetLinearVelocity(b, linearVelocity)
}

// Set the angular velocity of a body in radians per second
func (b BodyId) SetAngularVelocity(angularVelocity float32) {
	Body_SetAngularVelocity(b, angularVelocity)
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
	Body_SetTargetTransform(b, target, timeStep, wake)
}

// Get the linear velocity of a local point attached to a body. Usually in meters per second.
func (b BodyId) GetLocalPointVelocity(localPoint Vec2) Vec2 {
	return Body_GetLocalPointVelocity(b, localPoint)
}

// Get the linear velocity of a world point attached to a body. Usually in meters per second.
func (b BodyId) GetWorldPointVelocity(worldPoint Vec2) Vec2 {
	return Body_GetWorldPointVelocity(b, worldPoint)
}

// Apply a force at a world point. If the force is not applied at the center of mass,
// it will generate a torque and affect the angular velocity. This optionally wakes up the body.
// The force is ignored if the body is not awake.
// @param bodyId The body id
// @param force The world force vector, usually in newtons (N)
// @param point The world position of the point of application
// @param wake Option to wake up the body
func (b BodyId) ApplyForce(force Vec2, point Vec2, wake bool) {
	Body_ApplyForce(b, force, point, wake)
}

// Apply a force to the center of mass. This optionally wakes up the body.
// The force is ignored if the body is not awake.
// @param bodyId The body id
// @param force the world force vector, usually in newtons (N).
// @param wake also wake up the body
func (b BodyId) ApplyForceToCenter(force Vec2, wake bool) {
	Body_ApplyForceToCenter(b, force, wake)
}

// Apply a torque. This affects the angular velocity without affecting the linear velocity.
// This optionally wakes the body. The torque is ignored if the body is not awake.
// @param bodyId The body id
// @param torque about the z-axis (out of the screen), usually in N*m.
// @param wake also wake up the body
func (b BodyId) ApplyTorque(torque float32, wake bool) {
	Body_ApplyTorque(b, torque, wake)
}

// Clear the force and torque on this body. Forces and torques are automatically cleared after each world
// step. So this only needs to be called if the application wants to remove the effect of previous
// calls to apply forces and torques before the world step is called.
// @param bodyId The body id
func (b BodyId) ClearForces() {
	Body_ClearForces(b)
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
	Body_ApplyLinearImpulse(b, impulse, point, wake)
}

// Apply an impulse to the center of mass. This immediately modifies the velocity.
// The impulse is ignored if the body is not awake. This optionally wakes the body.
// @param bodyId The body id
// @param impulse the world impulse vector, usually in N*s or kg*m/s.
// @param wake also wake up the body
// @warning This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
func (b BodyId) ApplyLinearImpulseToCenter(impulse Vec2, wake bool) {
	Body_ApplyLinearImpulseToCenter(b, impulse, wake)
}

// Apply an angular impulse. The impulse is ignored if the body is not awake.
// This optionally wakes the body.
// @param bodyId The body id
// @param impulse the angular impulse, usually in units of kg*m*m/s
// @param wake also wake up the body
// @warning This should be used for one-shot impulses. If you need a steady torque,
// use a torque instead, which will work better with the sub-stepping solver.
func (b BodyId) ApplyAngularImpulse(impulse float32, wake bool) {
	Body_ApplyAngularImpulse(b, impulse, wake)
}

// Get the mass of the body, usually in kilograms
func (b BodyId) GetMass() float32 {
	return Body_GetMass(b)
}

// Get the rotational inertia of the body, usually in kg*m^2
func (b BodyId) GetRotationalInertia() float32 {
	return Body_GetRotationalInertia(b)
}

// Get the center of mass position of the body in local space
func (b BodyId) GetLocalCenterOfMass() Vec2 {
	return Body_GetLocalCenterOfMass(b)
}

// Get the center of mass position of the body in world space
func (b BodyId) GetWorldCenterOfMass() Vec2 {
	return Body_GetWorldCenterOfMass(b)
}

// Override the body's mass properties. Normally this is computed automatically using the
// shape geometry and density. This information is lost if a shape is added or removed or if the
// body type changes.
func (b BodyId) SetMassData(massData MassData) {
	Body_SetMassData(b, massData)
}

// Get the mass data for a body
func (b BodyId) GetMassData() MassData {
	return Body_GetMassData(b)
}

// This updates the mass properties to the sum of the mass properties of the shapes.
// This normally does not need to be called unless you called SetMassData to override
// the mass and you later want to reset the mass.
// You may also use this when automatic mass computation has been disabled.
// You should call this regardless of body type.
// Note that sensor shapes may have mass.
func (b BodyId) ApplyMassFromShapes() {
	Body_ApplyMassFromShapes(b)
}

// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
func (b BodyId) SetLinearDamping(linearDamping float32) {
	Body_SetLinearDamping(b, linearDamping)
}

// Get the current linear damping.
func (b BodyId) GetLinearDamping() float32 {
	return Body_GetLinearDamping(b)
}

// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
func (b BodyId) SetAngularDamping(angularDamping float32) {
	Body_SetAngularDamping(b, angularDamping)
}

// Get the current angular damping.
func (b BodyId) GetAngularDamping() float32 {
	return Body_GetAngularDamping(b)
}

// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
// @see b2BodyDef::gravityScale
func (b BodyId) SetGravityScale(gravityScale float32) {
	Body_SetGravityScale(b, gravityScale)
}

// Get the current gravity scale
func (b BodyId) GetGravityScale() float32 {
	return Body_GetGravityScale(b)
}

// @return true if this body is awake
func (b BodyId) IsAwake() bool {
	return Body_IsAwake(b)
}

// Wake a body from sleep. This wakes the entire island the body is touching.
// @warning Putting a body to sleep will put the entire island of bodies touching this body to sleep,
// which can be expensive and possibly unintuitive.
func (b BodyId) SetAwake(awake bool) {
	Body_SetAwake(b, awake)
}

// Wake bodies touching this body. Works for static bodies.
func (b BodyId) WakeTouching() {
	Body_WakeTouching(b)
}

// Enable or disable sleeping for this body. If sleeping is disabled the body will wake.
func (b BodyId) EnableSleep(enableSleep bool) {
	Body_EnableSleep(b, enableSleep)
}

// Returns true if sleeping is enabled for this body
func (b BodyId) IsSleepEnabled() bool {
	return Body_IsSleepEnabled(b)
}

// Set the sleep threshold, usually in meters per second
func (b BodyId) SetSleepThreshold(sleepThreshold float32) {
	Body_SetSleepThreshold(b, sleepThreshold)
}

// Get the sleep threshold, usually in meters per second.
func (b BodyId) GetSleepThreshold() float32 {
	return Body_GetSleepThreshold(b)
}

// Returns true if this body is enabled
func (b BodyId) IsEnabled() bool {
	return Body_IsEnabled(b)
}

// Disable a body by removing it completely from the simulation. This is expensive.
func (b BodyId) Disable() {
	Body_Disable(b)
}

// Enable a body by adding it to the simulation. This is expensive.
func (b BodyId) Enable() {
	Body_Enable(b)
}

// Set the motion locks on this body.
func (b BodyId) SetMotionLocks(locks MotionLocks) {
	Body_SetMotionLocks(b, locks)
}

// Get the motion locks for this body.
func (b BodyId) GetMotionLocks() MotionLocks {
	return Body_GetMotionLocks(b)
}

// Set this body to be a bullet. A bullet does continuous collision detection
// against dynamic bodies (but not other bullets).
func (b BodyId) SetBullet(flag bool) {
	Body_SetBullet(b, flag)
}

// Is this body a bullet?
func (b BodyId) IsBullet() bool {
	return Body_IsBullet(b)
}

// Enable/disable contact events on all shapes.
// @see b2ShapeDef::enableContactEvents
// @warning changing this at runtime may cause mismatched begin/end touch events
func (b BodyId) EnableContactEvents(flag bool) {
	Body_EnableContactEvents(b, flag)
}

// Enable/disable hit events on all shapes
// @see b2ShapeDef::enableHitEvents
func (b BodyId) EnableHitEvents(flag bool) {
	Body_EnableHitEvents(b, flag)
}

// Get the world that owns this body
func (b BodyId) GetWorld() WorldId {
	return Body_GetWorld(b)
}

// Get the number of shapes on this body
func (b BodyId) GetShapeCount() int {
	return Body_GetShapeCount(b)
}

// Get the shape ids for all shapes on this body, up to the provided capacity.
// @returns the number of shape ids stored in the user array
func (b BodyId) GetShapes(shapeArray *ShapeId, capacity int) int {
	return Body_GetShapes(b, shapeArray, capacity)
}

// Get the number of joints on this body
func (b BodyId) GetJointCount() int {
	return Body_GetJointCount(b)
}

// Get the joint ids for all joints on this body, up to the provided capacity
// @returns the number of joint ids stored in the user array
func (b BodyId) GetJoints(jointArray *JointId, capacity int) int {
	return Body_GetJoints(b, jointArray, capacity)
}

// Get the maximum capacity required for retrieving all the touching contacts on a body
func (b BodyId) GetContactCapacity() int {
	return Body_GetContactCapacity(b)
}

// Get the touching contact data for a body.
// @note Box2D uses speculative collision so some contact points may be separated.
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
func (b BodyId) GetContactData(contactData *ContactData, capacity int) int {
	return Body_GetContactData(b, contactData, capacity)
}

// Get the current world AABB that contains all the attached shapes. Note that this may not encompass the body origin.
// If there are no shapes attached then the returned AABB is empty and centered on the body origin.
func (b BodyId) ComputeAABB() AABB {
	return Body_ComputeAABB(b)
}

// Shape id references a shape instance. This should be treated as an opaque handle.
type ShapeId struct {
	index1     int32
	world0     uint16
	generation uint16
}

func (s ShapeId) Defer() {
	s.Destroy(false)
}

// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
// Do not keep references to the associated shapes and joints.
func (s ShapeId) Destroy(updateBodyMass bool) {
	DestroyShape(s, updateBodyMass)
}

// Shape identifier validation. Provides validation for up to 64K allocations.
func (s ShapeId) IsValid() bool {
	return Shape_IsValid(s)
}

// Get the type of a shape
func (s ShapeId) GetType() ShapeType {
	return Shape_GetType(s)
}

// Get the id of the body that a shape is attached to
func (s ShapeId) GetBody() BodyId {
	return Shape_GetBody(s)
}

// Get the world that owns this shape
func (s ShapeId) GetWorld() WorldId {
	return Shape_GetWorld(s)
}

// Returns true if the shape is a sensor. It is not possible to change a shape
// from sensor to solid dynamically because this breaks the contract for
// sensor events.
func (s ShapeId) IsSensor() bool {
	return Shape_IsSensor(s)
}

// Set the mass density of a shape, usually in kg/m^2.
// This will optionally update the mass properties on the parent body.
// @see b2ShapeDef::density, b2Body_ApplyMassFromShapes
func (s ShapeId) SetDensity(density float32, updateBodyMass bool) {
	Shape_SetDensity(s, density, updateBodyMass)
}

// Get the density of a shape, usually in kg/m^2
func (s ShapeId) GetDensity() float32 {
	return Shape_GetDensity(s)
}

// Set the friction on a shape
func (s ShapeId) SetFriction(friction float32) {
	Shape_SetFriction(s, friction)
}

// Get the friction of a shape
func (s ShapeId) GetFriction() float32 {
	return Shape_GetFriction(s)
}

// Set the shape restitution (bounciness)
func (s ShapeId) SetRestitution(restitution float32) {
	Shape_SetRestitution(s, restitution)
}

// Get the shape restitution
func (s ShapeId) GetRestitution() float32 {
	return Shape_GetRestitution(s)
}

// Set the user material identifier
func (s ShapeId) SetUserMaterial(material uint64) {
	Shape_SetUserMaterial(s, material)
}

// Get the user material identifier
func (s ShapeId) GetUserMaterial() uint64 {
	return Shape_GetUserMaterial(s)
}

// Set the shape surface material
func (s ShapeId) SetSurfaceMaterial(surfaceMaterial *SurfaceMaterial) {
	Shape_SetSurfaceMaterial(s, surfaceMaterial)
}

// Get the shape surface material
func (s ShapeId) GetSurfaceMaterial() SurfaceMaterial {
	return Shape_GetSurfaceMaterial(s)
}

// Get the shape filter
func (s ShapeId) GetFilter() Filter {
	return Shape_GetFilter(s)
}

// Set the current filter. This is almost as expensive as recreating the shape. This may cause
// contacts to be immediately destroyed. However contacts are not created until the next world step.
// Sensor overlap state is also not updated until the next world step.
// @see b2ShapeDef::filter
func (s ShapeId) SetFilter(filter Filter) {
	Shape_SetFilter(s, filter)
}

// Enable sensor events for this shape.
// @see b2ShapeDef::enableSensorEvents
func (s ShapeId) EnableSensorEvents(flag bool) {
	Shape_EnableSensorEvents(s, flag)
}

// Returns true if sensor events are enabled.
func (s ShapeId) AreSensorEventsEnabled() bool {
	return Shape_AreSensorEventsEnabled(s)
}

// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
// @see b2ShapeDef::enableContactEvents
// @warning changing this at run-time may lead to lost begin/end events
func (s ShapeId) EnableContactEvents(flag bool) {
	Shape_EnableContactEvents(s, flag)
}

// Returns true if contact events are enabled
func (s ShapeId) AreContactEventsEnabled() bool {
	return Shape_AreContactEventsEnabled(s)
}

// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
// and must be carefully handled due to multithreading. Ignored for sensors.
// @see b2PreSolveFcn
func (s ShapeId) EnablePreSolveEvents(flag bool) {
	Shape_EnablePreSolveEvents(s, flag)
}

// Returns true if pre-solve events are enabled
func (s ShapeId) ArePreSolveEventsEnabled() bool {
	return Shape_ArePreSolveEventsEnabled(s)
}

// Enable contact hit events for this shape. Ignored for sensors.
// @see b2WorldDef.hitEventThreshold
func (s ShapeId) EnableHitEvents(flag bool) {
	Shape_EnableHitEvents(s, flag)
}

// Returns true if hit events are enabled
func (s ShapeId) AreHitEventsEnabled() bool {
	return Shape_AreHitEventsEnabled(s)
}

// Test a point for overlap with a shape
func (s ShapeId) TestPoint(point Vec2) bool {
	return Shape_TestPoint(s, point)
}

// Ray cast a shape directly
func (s ShapeId) RayCast(input RayCastInput) CastOutput {
	return Shape_RayCast(s, input)
}

// Get a copy of the shape's circle. Asserts the type is correct.
func (s ShapeId) GetCircle() Circle {
	return Shape_GetCircle(s)
}

// Get a copy of the shape's line segment. Asserts the type is correct.
func (s ShapeId) GetSegment() Segment {
	return Shape_GetSegment(s)
}

// Get a copy of the shape's chain segment. These come from chain shapes.
// Asserts the type is correct.
func (s ShapeId) GetChainSegment() ChainSegment {
	return Shape_GetChainSegment(s)
}

// Get a copy of the shape's capsule. Asserts the type is correct.
func (s ShapeId) GetCapsule() Capsule {
	return Shape_GetCapsule(s)
}

// Get a copy of the shape's convex polygon. Asserts the type is correct.
func (s ShapeId) GetPolygon() Polygon {
	return Shape_GetPolygon(s)
}

// Allows you to change a shape to be a circle or update the current circle.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func (s ShapeId) SetCircle(circle Circle) {
	Shape_SetCircle(s, circle)
}

// Allows you to change a shape to be a capsule or update the current capsule.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func (s ShapeId) SetCapsule(capsule Capsule) {
	Shape_SetCapsule(s, capsule)
}

// Allows you to change a shape to be a segment or update the current segment.
func (s ShapeId) SetSegment(segment Segment) {
	Shape_SetSegment(s, segment)
}

// Allows you to change a shape to be a polygon or update the current polygon.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func (s ShapeId) SetPolygon(polygon Polygon) {
	Shape_SetPolygon(s, polygon)
}

// Get the parent chain id if the shape type is a chain segment, otherwise
// returns b2_nullChainId.
func (s ShapeId) GetParentChain() ChainId {
	return Shape_GetParentChain(s)
}

// Get the maximum capacity required for retrieving all the touching contacts on a shape
func (s ShapeId) GetContactCapacity() int {
	return Shape_GetContactCapacity(s)
}

// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
// @note Box2D uses speculative collision so some contact points may be separated.
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
func (s ShapeId) GetContactData(contactData *ContactData, capacity int) int {
	return Shape_GetContactData(s, contactData, capacity)
}

// Get the maximum capacity required for retrieving all the overlapped shapes on a sensor shape.
// This returns 0 if the provided shape is not a sensor.
// @param shapeId the id of a sensor shape
// @returns the required capacity to get all the overlaps in b2Shape_GetSensorData
func (s ShapeId) GetSensorCapacity() int {
	return Shape_GetSensorCapacity(s)
}

// Get the overlap data for a sensor shape computed the previous world step.
// @param shapeId the id of a sensor shape
// @param visitorIds a user allocated array that is filled with the overlapping shapes (visitors)
// @param capacity the capacity of overlappedShapes
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
// @warning overlaps may contain destroyed shapes so use b2Shape_IsValid to confirm each overlap
func (s ShapeId) GetSensorData(visitorIds *ShapeId, capacity int) int {
	return Shape_GetSensorData(s, visitorIds, capacity)
}

// Get the current world AABB
func (s ShapeId) GetAABB() AABB {
	return Shape_GetAABB(s)
}

// Compute the mass data for a shape
func (s ShapeId) ComputeMassData() MassData {
	return Shape_ComputeMassData(s)
}

// Get the closest point on a shape to a target point. Target and result are in world space.
// todo need sample
func (s ShapeId) GetClosestPoint(target Vec2) Vec2 {
	return Shape_GetClosestPoint(s, target)
}

// Apply a wind force to the body for this shape using the density of air. This considers
// the projected area of the shape in the wind direction. This also considers
// the relative velocity of the shape.
// @param shapeId the shape id
// @param wind the wind velocity in world space
// @param drag the drag coefficient, the force that opposes the relative velocity
// @param lift the lift coefficient, the force that is perpendicular to the relative velocity
// @param wake should this wake the body
func (s ShapeId) ApplyWind(wind Vec2, drag float32, lift float32, wake bool) {
	Shape_ApplyWind(s, wind, drag, lift, wake)
}

// Chain id references a chain instances. This should be treated as an opaque handle.
type ChainId struct {
	index1     int32
	world0     uint16
	generation uint16
}

// Joint id references a joint instance. This should be treated as an opaque handle.
type JointId struct {
	index1     int32
	world0     uint16
	generation uint16
}

// Contact id references a contact instance. This should be treated as an opaque handled.
type ContactId struct {
	index1     int32
	world0     uint16
	padding    int16
	generation uint32
}
