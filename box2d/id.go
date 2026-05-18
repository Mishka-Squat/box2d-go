package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

import "unsafe"

// World id references a world instance. This should be treated as an opaque handle.
type WorldId struct {
	index1     uint16
	generation uint16
}

// goworldidptr - Returns new WorldId from pointer
func goworldidptr(ptr *C.b2WorldId) *WorldId {
	return (*WorldId)(unsafe.Pointer(ptr))
}

// cworldidptr returns b2WorldId C pointer
func cworldidptr(col *WorldId) *C.b2WorldId {
	return (*C.b2WorldId)(unsafe.Pointer(col))
}

// Body id references a body instance. This should be treated as an opaque handle.
type BodyId struct {
	index1     int32
	world0     uint16
	generation uint16
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
