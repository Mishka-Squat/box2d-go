package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

//
// @defgroup contact Contact
// Access to contacts
//

// Contact id references a contact instance. This should be treated as an opaque handled.
type ContactId struct {
	index1     int32
	world0     uint16
	padding    int16
	generation uint32
}

// Contact identifier validation. Provides validation for up to 2^32 allocations.
func (c ContactId) IsValid() bool {
	return bool(C.b2Contact_IsValid(*cast[C.b2ContactId](&c)))
}

// Get the data for a contact. The manifold may have no points if the contact is not touching.
func (c ContactId) GetData() ContactData {
	r := C.b2Contact_GetData(*cast[C.b2ContactId](&c))
	return *cast[ContactData](&r)
}
