package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

func DefaultWorldDef() WorldDef {
	r := C.b2DefaultWorldDef()
	return *goworlddefptr(&r)
}

func DefaultBodyDef() BodyDef {
	r := C.b2DefaultBodyDef()
	return *gobodydefptr(&r)
}

func CreateWorld(def *WorldDef) WorldId {
	cdef := cworlddefptr(def)
	r := C.b2CreateWorld(cdef)
	return *goworldidptr(&r)
}

// Destroy a world
func DestroyWorld(id WorldId) {
	C.b2DestroyWorld(*cworldidptr(&id))
}

// World id validation. Provides validation for up to 64K allocations.
func World_IsValid(id WorldId) bool {
	r := C.b2World_IsValid(*cworldidptr(&id))
	return bool(r)
}

func (w WorldId) IsValid() bool {
	return World_IsValid(w)
}

func CreateBody(world WorldId, def *BodyDef) BodyId {
	cworld := *cworldidptr(&world)
	cdef := cbodydefptr(def)
	r := C.b2CreateBody(cworld, cdef)
	return *gobodyidptr(&r)
}

func DestroyBody(id BodyId) {
	C.b2DestroyBody(*cbodyidptr(&id))
}

func (w WorldId) CreateBody(def *BodyDef) BodyId {
	return CreateBody(w, def)
}

func Body_IsValid(id BodyId) bool {
	r := C.b2Body_IsValid(*cbodyidptr(&id))
	return bool(r)
}

func (b BodyId) IsValid() bool {
	return Body_IsValid(b)
}
