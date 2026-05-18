package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

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
	r := C.b2World_IsValid(*cast[C.b2WorldId](&id))
	return bool(r)
}

func World_Step(id WorldId, timeStep float32, subStepCount int) {
	C.b2World_Step(*cast[C.b2WorldId](&id), C.float(timeStep), C.int(subStepCount))
}

func World_Draw(id WorldId, draw *DebugDraw) {
	C.b2World_Draw(*cast[C.b2WorldId](&id), cast[C.b2DebugDraw](draw))
}

func b2World_GetBodyEvents(id WorldId) BodyEvents {
	r := C.b2World_GetBodyEvents(*cast[C.b2WorldId](&id))
	return *cast[BodyEvents](&r)
}

func b2World_GetSensorEvents(id WorldId) SensorEvents {
	r := C.b2World_GetSensorEvents(*cast[C.b2WorldId](&id))
	return *cast[SensorEvents](&r)
}

func b2World_GetContactEvents(id WorldId) ContactEvents {
	r := C.b2World_GetContactEvents(*cast[C.b2WorldId](&id))
	return *cast[ContactEvents](&r)
}

func b2World_GetJointEvents(id WorldId) JointEvents {
	r := C.b2World_GetJointEvents(*cast[C.b2WorldId](&id))
	return *cast[JointEvents](&r)
}

/*

func b2World_OverlapAABB(id WorldId, aabb b2AABB, filter QueryFilter, fcn b2OverlapResultFcn, context any) b2TreeStats {

}

func b2World_OverlapShape(id WorldId, proxy *b2ShapeProxy, filter QueryFilter, fcn b2OverlapResultFcn, context any) b2TreeStats {

}

*/

func CreateBody(world WorldId, def *BodyDef) BodyId {
	cworld := *cast[C.b2WorldId](&world)
	cdef := cast[C.b2BodyDef](def)
	r := C.b2CreateBody(cworld, cdef)
	return *cast[BodyId](&r)
}

func DestroyBody(id BodyId) {
	C.b2DestroyBody(*cast[C.b2BodyId](&id))
}

func Body_IsValid(id BodyId) bool {
	r := C.b2Body_IsValid(*cast[C.b2BodyId](&id))
	return bool(r)
}
