package box2d

/*
#include "box2d/box2d.h"
*/
import "C"

// Chain id references a chain instances. This should be treated as an opaque handle.
type ChainId struct {
	index1     int32
	world0     uint16
	generation uint16
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
*/
