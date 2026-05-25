package box2d

/*
#include "box2d/box2d.h"
*/
import "C"

// Shape id references a shape instance. This should be treated as an opaque handle.
type ShapeId struct {
	index1     int32
	world0     uint16
	generation uint16
}

func (s ShapeId) Defer() {
	s.Destroy(false)
}

// Destroy a shape. You may defer the body mass update which can improve performance if several shapes on a
//	body are destroyed at once.
//	@see b2Body_ApplyMassFromShapes
func (s ShapeId) Destroy(updateBodyMass bool) {
	C.b2DestroyShape(*cast[C.b2ShapeId](&s), C.bool(updateBodyMass))
}

// Shape identifier validation. Provides validation for up to 64K allocations.
func (s ShapeId) IsValid() bool {
	return bool(C.b2Shape_IsValid(*cast[C.b2ShapeId](&s)))
}

// Get the type of a shape
func (s ShapeId) GetType() ShapeType {
	return ShapeType(C.b2Shape_GetType(*cast[C.b2ShapeId](&s)))
}

// Get the id of the body that a shape is attached to
func (s ShapeId) GetBody() BodyId {
	r := C.b2Shape_GetBody(*cast[C.b2ShapeId](&s))
	return *cast[BodyId](&r)
}

// Get the world that owns this shape
func (s ShapeId) GetWorld() WorldId {
	r := C.b2Shape_GetWorld(*cast[C.b2ShapeId](&s))
	return *cast[WorldId](&r)
}

// Returns true if the shape is a sensor. It is not possible to change a shape
// from sensor to solid dynamically because this breaks the contract for
// sensor events.
func (s ShapeId) IsSensor() bool {
	return bool(C.b2Shape_IsSensor(*cast[C.b2ShapeId](&s)))
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
func (s ShapeId) SetDensity(density float32, updateBodyMass bool) {
	C.b2Shape_SetDensity(*cast[C.b2ShapeId](&s), C.float(density), C.bool(updateBodyMass))
}

// Get the density of a shape, usually in kg/m^2
func (s ShapeId) GetDensity() float32 {
	return float32(C.b2Shape_GetDensity(*cast[C.b2ShapeId](&s)))
}

// Set the friction on a shape
func (s ShapeId) SetFriction(friction float32) {
	C.b2Shape_SetFriction(*cast[C.b2ShapeId](&s), C.float(friction))
}

// Get the friction of a shape
func (s ShapeId) GetFriction() float32 {
	return float32(C.b2Shape_GetFriction(*cast[C.b2ShapeId](&s)))
}

// Set the shape restitution (bounciness)
func (s ShapeId) SetRestitution(restitution float32) {
	C.b2Shape_SetRestitution(*cast[C.b2ShapeId](&s), C.float(restitution))
}

// Get the shape restitution
func (s ShapeId) GetRestitution() float32 {
	return float32(C.b2Shape_GetRestitution(*cast[C.b2ShapeId](&s)))
}

// Set the user material identifier
func (s ShapeId) SetUserMaterial(material uint64) {
	C.b2Shape_SetUserMaterial(*cast[C.b2ShapeId](&s), C.uint64_t(material))
}

// Get the user material identifier
func (s ShapeId) GetUserMaterial() uint64 {
	return uint64(C.b2Shape_GetUserMaterial(*cast[C.b2ShapeId](&s)))
}

// Set the shape surface material
func (s ShapeId) SetSurfaceMaterial(surfaceMaterial *SurfaceMaterial) {
	C.b2Shape_SetSurfaceMaterial(*cast[C.b2ShapeId](&s), cast[C.b2SurfaceMaterial](surfaceMaterial))
}

// Get the shape surface material
func (s ShapeId) GetSurfaceMaterial() SurfaceMaterial {
	r := C.b2Shape_GetSurfaceMaterial(*cast[C.b2ShapeId](&s))
	return *cast[SurfaceMaterial](&r)
}

// Get the shape filter
func (s ShapeId) GetFilter() Filter {
	r := C.b2Shape_GetFilter(*cast[C.b2ShapeId](&s))
	return *cast[Filter](&r)
}

// Set the current filter. This is almost as expensive as recreating the shape. This may cause
// contacts to be immediately destroyed. However contacts are not created until the next world step.
// Sensor overlap state is also not updated until the next world step.
// @see b2ShapeDef::filter
func (s ShapeId) SetFilter(filter Filter) {
	C.b2Shape_SetFilter(*cast[C.b2ShapeId](&s), *cast[C.b2Filter](&filter))
}

// Enable sensor events for this shape.
// @see b2ShapeDef::enableSensorEvents
func (s ShapeId) EnableSensorEvents(flag bool) {
	C.b2Shape_EnableSensorEvents(*cast[C.b2ShapeId](&s), C.bool(flag))
}

// Returns true if sensor events are enabled.
func (s ShapeId) AreSensorEventsEnabled() bool {
	return bool(C.b2Shape_AreSensorEventsEnabled(*cast[C.b2ShapeId](&s)))
}

// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
// @see b2ShapeDef::enableContactEvents
// @warning changing this at run-time may lead to lost begin/end events
func (s ShapeId) EnableContactEvents(flag bool) {
	C.b2Shape_EnableContactEvents(*cast[C.b2ShapeId](&s), C.bool(flag))
}

// Returns true if contact events are enabled
func (s ShapeId) AreContactEventsEnabled() bool {
	return bool(C.b2Shape_AreContactEventsEnabled(*cast[C.b2ShapeId](&s)))
}

// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
// and must be carefully handled due to multithreading. Ignored for sensors.
// @see b2PreSolveFcn
func (s ShapeId) EnablePreSolveEvents(flag bool) {
	C.b2Shape_EnablePreSolveEvents(*cast[C.b2ShapeId](&s), C.bool(flag))
}

// Returns true if pre-solve events are enabled
func (s ShapeId) ArePreSolveEventsEnabled() bool {
	return bool(C.b2Shape_ArePreSolveEventsEnabled(*cast[C.b2ShapeId](&s)))
}

// Enable contact hit events for this shape. Ignored for sensors.
// @see b2WorldDef.hitEventThreshold
func (s ShapeId) EnableHitEvents(flag bool) {
	C.b2Shape_EnableHitEvents(*cast[C.b2ShapeId](&s), C.bool(flag))
}

// Returns true if hit events are enabled
func (s ShapeId) AreHitEventsEnabled() bool {
	return bool(C.b2Shape_AreHitEventsEnabled(*cast[C.b2ShapeId](&s)))
}

// Test a point for overlap with a shape
func (s ShapeId) TestPoint(point Vec2) bool {
	return bool(C.b2Shape_TestPoint(*cast[C.b2ShapeId](&s), *cast[C.b2Vec2](&point)))
}

// Ray cast a shape directly
func (s ShapeId) RayCast(input RayCastInput) CastOutput {
	r := C.b2Shape_RayCast(*cast[C.b2ShapeId](&s), cast[C.b2RayCastInput](&input))
	return *cast[CastOutput](&r)
}

// Get a copy of the shape's circle. Asserts the type is correct.
func (s ShapeId) GetCircle() Circle {
	r := C.b2Shape_GetCircle(*cast[C.b2ShapeId](&s))
	return *cast[Circle](&r)
}

// Get a copy of the shape's line segment. Asserts the type is correct.
func (s ShapeId) GetSegment() Segment {
	r := C.b2Shape_GetSegment(*cast[C.b2ShapeId](&s))
	return *cast[Segment](&r)
}

// Get a copy of the shape's chain segment. These come from chain shapes.
// Asserts the type is correct.
func (s ShapeId) GetChainSegment() ChainSegment {
	r := C.b2Shape_GetChainSegment(*cast[C.b2ShapeId](&s))
	return *cast[ChainSegment](&r)
}

// Get a copy of the shape's capsule. Asserts the type is correct.
func (s ShapeId) GetCapsule() Capsule {
	r := C.b2Shape_GetCapsule(*cast[C.b2ShapeId](&s))
	return *cast[Capsule](&r)
}

// Get a copy of the shape's convex polygon. Asserts the type is correct.
func (s ShapeId) GetPolygon() Polygon {
	r := C.b2Shape_GetPolygon(*cast[C.b2ShapeId](&s))
	return *cast[Polygon](&r)
}

// Allows you to change a shape to be a circle or update the current circle.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func (s ShapeId) SetCircle(circle Circle) {
	C.b2Shape_SetCircle(*cast[C.b2ShapeId](&s), cast[C.b2Circle](&circle))
}

// Allows you to change a shape to be a capsule or update the current capsule.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func (s ShapeId) SetCapsule(capsule Capsule) {
	C.b2Shape_SetCapsule(*cast[C.b2ShapeId](&s), cast[C.b2Capsule](&capsule))
}

// Allows you to change a shape to be a segment or update the current segment.
func (s ShapeId) SetSegment(segment Segment) {
	C.b2Shape_SetSegment(*cast[C.b2ShapeId](&s), cast[C.b2Segment](&segment))
}

// Allows you to change a shape to be a polygon or update the current polygon.
// This does not modify the mass properties.
// @see b2Body_ApplyMassFromShapes
func (s ShapeId) SetPolygon(polygon Polygon) {
	C.b2Shape_SetPolygon(*cast[C.b2ShapeId](&s), cast[C.b2Polygon](&polygon))
}

// Get the parent chain id if the shape type is a chain segment, otherwise
// returns b2_nullChainId.
func (s ShapeId) GetParentChain() ChainId {
	r := C.b2Shape_GetParentChain(*cast[C.b2ShapeId](&s))
	return *cast[ChainId](&r)
}

// Get the maximum capacity required for retrieving all the touching contacts on a shape
func (s ShapeId) GetContactCapacity() int {
	return int(C.b2Shape_GetContactCapacity(*cast[C.b2ShapeId](&s)))
}

// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
// @note Box2D uses speculative collision so some contact points may be separated.
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
func (s ShapeId) GetContactData(contactData *ContactData, capacity int) int {
	return int(C.b2Shape_GetContactData(*cast[C.b2ShapeId](&s), cast[C.b2ContactData](contactData), C.int(capacity)))
}

// Get the maximum capacity required for retrieving all the overlapped shapes on a sensor shape.
// This returns 0 if the provided shape is not a sensor.
// @param shapeId the id of a sensor shape
// @returns the required capacity to get all the overlaps in b2Shape_GetSensorData
func (s ShapeId) GetSensorCapacity() int {
	return int(C.b2Shape_GetSensorCapacity(*cast[C.b2ShapeId](&s)))
}

// Get the overlap data for a sensor shape computed the previous world step.
// @param shapeId the id of a sensor shape
// @param visitorIds a user allocated array that is filled with the overlapping shapes (visitors)
// @param capacity the capacity of overlappedShapes
// @returns the number of elements filled in the provided array
// @warning do not ignore the return value, it specifies the valid number of elements
// @warning overlaps may contain destroyed shapes so use b2Shape_IsValid to confirm each overlap
func (s ShapeId) GetSensorData(visitorIds *ShapeId, capacity int) int {
	return int(C.b2Shape_GetSensorData(*cast[C.b2ShapeId](&s), cast[C.b2ShapeId](visitorIds), C.int(capacity)))
}

// Get the current world AABB
func (s ShapeId) GetAABB() AABB {
	r := C.b2Shape_GetAABB(*cast[C.b2ShapeId](&s))
	return *cast[AABB](&r)
}

// Compute the mass data for a shape
func (s ShapeId) ComputeMassData() MassData {
	r := C.b2Shape_ComputeMassData(*cast[C.b2ShapeId](&s))
	return *cast[MassData](&r)
}

// Get the closest point on a shape to a target point. Target and result are in world space.
// todo need sample
func (s ShapeId) GetClosestPoint(target Vec2) Vec2 {
	r := C.b2Shape_GetClosestPoint(*cast[C.b2ShapeId](&s), *cast[C.b2Vec2](&target))
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
func (s ShapeId) ApplyWind(wind Vec2, drag float32, lift float32, wake bool) {
	C.b2Shape_ApplyWind(*cast[C.b2ShapeId](&s), *cast[C.b2Vec2](&wind), C.float(drag), C.float(lift), C.bool(wake))
}
