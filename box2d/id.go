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

func (b BodyId) Destroy() {
	DestroyBody(b)
}

func (b BodyId) Defer() {
	b.Destroy()
}

func (b BodyId) IsValid() bool {
	return Body_IsValid(b)
}

func (b BodyId) GetPosition() Vec2 {
	return Body_GetPosition(b)
}

// Get the world rotation of a body as a cosine/sine pair (complex number)
func (b BodyId) GetRotation() Rot {
	return Body_GetRotation(b)
}

// Shape id references a shape instance. This should be treated as an opaque handle.
type ShapeId struct {
	index1     int32
	world0     uint16
	generation uint16
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
