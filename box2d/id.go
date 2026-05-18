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
