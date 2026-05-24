package box2d

/*
#include "box2d/box2d.h"
#include <stdlib.h>
*/
import "C"

/*

type b2SimplexCache b2SimplexCache; struct type b2Hull b2Hull; struct
*/
//
// @defgroup geometry Geometry
// @brief Geometry types and algorithms
//
// Definitions of circles, capsules, segments, and polygons. Various algorithms to compute hulls, mass properties, and so on.
// Functions should take the shape as the first argument to assist editor auto-complete.
//

// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
// don't use more vertices.
const B2_MAX_POLYGON_VERTICES = 8

// Low level ray cast input data
type RayCastInput struct {
	// Start point of the ray cast
	Origin Vec2

	// Translation of the ray cast
	Translation Vec2

	// The maximum fraction of the translation to consider, typically 1
	MaxFraction float32
}

// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
// You can provide between 1 and B2_MAX_POLYGON_VERTICES and a radius.
type ShapeProxy struct {
	// The point cloud
	Points [B2_MAX_POLYGON_VERTICES]Vec2

	// The number of points. Must be greater than 0.
	Count int32

	// The external radius of the point cloud. May be zero.
	Radius float32
}

// Low level shape cast input in generic form. This allows casting an arbitrary point
// cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
type ShapeCastInput struct {
	// A generic shape
	Proxy ShapeProxy

	// The translation of the shape cast
	Translation Vec2

	// The maximum fraction of the translation to consider, typically 1
	MaxFraction float32

	// Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.
	CanEncroach bool
}

// Low level ray cast or shape-cast output data. Returns a zero fraction and normal in the case of initial overlap.
type CastOutput struct {
	// The surface normal at the hit point
	Normal Vec2

	// The surface hit point
	Point Vec2

	// The fraction of the input translation at collision
	Fraction float32

	// The number of iterations used
	Iterations int32

	// Did the cast hit?
	Hit bool
}

// This holds the mass data computed for a shape.
type MassData struct {
	// The mass of the shape, usually in kilograms.
	Mass float32

	// The position of the shape's centroid relative to the shape's origin.
	Center Vec2

	// The rotational inertia of the shape about the shape center.
	RotationalInertia float32
}

// A solid circle
type Circle struct {
	// The local center
	Center Vec2

	// The radius
	Radius float32
}

// A solid capsule can be viewed as two semicircles connected
// by a rectangle.
type Capsule struct {
	// Local center of the first semicircle
	Center1 Vec2

	// Local center of the second semicircle
	Center2 Vec2

	// The radius of the semicircles
	Radius float32
}

// A solid convex polygon. It is assumed that the interior of the polygon is to
// the left of each edge.
// Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES.
// In most cases you should not need many vertices for a convex polygon.
// @warning DO NOT fill this out manually, instead use a helper function like
// b2MakePolygon or b2MakeBox.
type Polygon struct {
	// The polygon vertices
	Vertices [B2_MAX_POLYGON_VERTICES]Vec2

	// The outward normal vectors of the polygon sides
	Normals [B2_MAX_POLYGON_VERTICES]Vec2

	// The centroid of the polygon
	Centroid Vec2

	// The external radius for rounded polygons
	Radius float32

	// The number of polygon vertices
	Count int32
}

// A line segment with two-sided collision.
type Segment struct {
	// The first point
	Point1 Vec2

	// The second point
	Point2 Vec2
}

// A line segment with one-sided collision. Only collides on the right side.
// Several of these are generated for a chain shape.
// ghost1 -> point1 -> point2 -> ghost2
type ChainSegment struct {
	// The tail ghost vertex
	Ghost1 Vec2

	// The line segment
	Segment Segment

	// The head ghost vertex
	Ghost2 Vec2

	// The owning chain shape index (internal usage only)
	chainId int32
}

/*
// Validate ray cast input data (NaN, etc)
B2_API bool b2IsValidRay( const RayCastInput* input );
*/

func MakeCircle(radius float32) Circle {
	return Circle{
		Radius: radius,
	}
}

func MakeCircleOffset(center Vec2, radius float32) Circle {
	return Circle{
		Center: center,
		Radius: radius,
	}
}

/*
// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API Polygon b2MakePolygon( const b2Hull* hull, float32 radius );

// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API Polygon b2MakeOffsetPolygon( const b2Hull* hull, Vec2 position, Rot rotation );

// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API Polygon b2MakeOffsetRoundedPolygon( const b2Hull* hull, Vec2 position, Rot rotation, float32 radius );

*/
// Make a square polygon, bypassing the need for a convex hull.
// @param halfWidth the half-width
func MakeSquare(halfWidth float32) Polygon {
	return MakeBox(halfWidth, halfWidth)
}

// Make a box (rectangle) polygon, bypassing the need for a convex hull.
// @param halfWidth the half-width (x-axis)
// @param halfHeight the half-height (y-axis)
func MakeBox(halfWidth float32, halfHeight float32) Polygon {
	shape := Polygon{
		Vertices: [B2_MAX_POLYGON_VERTICES]Vec2{
			{X: -halfWidth, Y: -halfHeight},
			{X: halfWidth, Y: -halfHeight},
			{X: halfWidth, Y: halfHeight},
			{X: -halfWidth, Y: halfHeight},
		},
		Normals: [B2_MAX_POLYGON_VERTICES]Vec2{
			{X: 0.0, Y: -1.0},
			{X: 1.0, Y: 0.0},
			{X: 0.0, Y: 1.0},
			{X: -1.0, Y: 0.0},
		},
		Centroid: Vec2{X: 0, Y: 0},
		Radius:   0,
		Count:    4,
	}

	return shape
}

// Make a rounded box, bypassing the need for a convex hull.
// @param halfWidth the half-width (x-axis)
// @param halfHeight the half-height (y-axis)
// @param radius the radius of the rounded extension
func MakeRoundedBox(halfWidth float32, halfHeight float32, radius float32) Polygon {
	shape := MakeBox(halfWidth, halfHeight)
	shape.Radius = radius
	return shape
}

/*
// Make an offset box, bypassing the need for a convex hull.
// @param halfWidth the half-width (x-axis)
// @param halfHeight the half-height (y-axis)
// @param center the local center of the box
// @param rotation the local rotation of the box
func MakeOffsetBox(halfWidth float32, halfHeight float32, center Vec2, rotation Rot) Polygon {
	shape := Polygon{
		Vertices: [B2_MAX_POLYGON_VERTICES]Vec2{
			{X: -halfWidth, Y: -halfHeight},
			{X: halfWidth, Y: -halfHeight},
			{X: halfWidth, Y: halfHeight},
			{X: -halfWidth, Y: halfHeight},
		},
		Normals: [B2_MAX_POLYGON_VERTICES]Vec2{
			{X: 0.0, Y: -1.0},
			{X: 1.0, Y: 0.0},
			{X: 0.0, Y: 1.0},
			{X: -1.0, Y: 0.0},
		},
		Centroid: Vec2{X: 0, Y: 0},
		Radius:   0,
		Count:    4,
	}

	return shape
}

// Make an offset rounded box, bypassing the need for a convex hull.
// @param halfWidth the half-width (x-axis)
// @param halfHeight the half-height (y-axis)
// @param center the local center of the box
// @param rotation the local rotation of the box
// @param radius the radius of the rounded extension
func MakeOffsetRoundedBox(halfWidth float32, halfHeight float32, center Vec2, rotation Rot, radius float32) Polygon {
	shape := MakeOffsetBox(halfWidth, halfHeight, center, rotation)
	shape.Radius = radius
	return shape
}

// Transform a polygon. This is useful for transferring a shape from one body to another.
B2_API Polygon b2TransformPolygon( b2Transform transform, const Polygon* polygon );

// Compute mass properties of a circle
B2_API b2MassData b2ComputeCircleMass( const b2Circle* shape, float32 density );

// Compute mass properties of a capsule
B2_API b2MassData b2ComputeCapsuleMass( const b2Capsule* shape, float32 density );

// Compute mass properties of a polygon
B2_API b2MassData b2ComputePolygonMass( const Polygon* shape, float32 density );

// Compute the bounding box of a transformed circle
B2_API AABB b2ComputeCircleAABB( const b2Circle* shape, b2Transform transform );

// Compute the bounding box of a transformed capsule
B2_API AABB b2ComputeCapsuleAABB( const b2Capsule* shape, b2Transform transform );

// Compute the bounding box of a transformed polygon
B2_API AABB b2ComputePolygonAABB( const Polygon* shape, b2Transform transform );

// Compute the bounding box of a transformed line segment
B2_API AABB b2ComputeSegmentAABB( const b2Segment* shape, b2Transform transform );

// Test a point for overlap with a circle in local space
B2_API bool b2PointInCircle( const b2Circle* shape, Vec2 point );

// Test a point for overlap with a capsule in local space
B2_API bool b2PointInCapsule( const b2Capsule* shape, Vec2 point );

// Test a point for overlap with a convex polygon in local space
B2_API bool b2PointInPolygon( const Polygon* shape, Vec2 point );

// Ray cast versus circle shape in local space.
B2_API b2CastOutput b2RayCastCircle( const b2Circle* shape, const RayCastInput* input );

// Ray cast versus capsule shape in local space.
B2_API b2CastOutput b2RayCastCapsule( const b2Capsule* shape, const RayCastInput* input );

// Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with hits from
// the left side being treated as a miss.
B2_API b2CastOutput b2RayCastSegment( const b2Segment* shape, const RayCastInput* input, bool oneSided );

// Ray cast versus polygon shape in local space.
B2_API b2CastOutput b2RayCastPolygon( const Polygon* shape, const RayCastInput* input );

// Shape cast versus a circle.
B2_API b2CastOutput b2ShapeCastCircle(const b2Circle* shape,  const ShapeCastInput* input );

// Shape cast versus a capsule.
B2_API b2CastOutput b2ShapeCastCapsule( const b2Capsule* shape, const ShapeCastInput* input);

// Shape cast versus a line segment.
B2_API b2CastOutput b2ShapeCastSegment( const b2Segment* shape, const ShapeCastInput* input );

// Shape cast versus a convex polygon.
B2_API b2CastOutput b2ShapeCastPolygon( const Polygon* shape, const ShapeCastInput* input );
*/
// A convex hull. Used to create convex polygons.
// @warning Do not modify these values directly, instead use b2ComputeHull()
type Hull struct {
	// The final points of the hull
	Points [B2_MAX_POLYGON_VERTICES]Vec2

	// The number of points
	Count int32
}

/*
// Compute the convex hull of a set of points. Returns an empty hull if it fails.
// Some failure cases:
// - all points very close together
// - all points on a line
// - less than 3 points
// - more than B2_MAX_POLYGON_VERTICES points
// This welds close points and removes collinear points.
// @warning Do not modify a hull once it has been computed
B2_API b2Hull b2ComputeHull( const Vec2* points, int count );

// This determines if a hull is valid. Checks for:
// - convexity
// - collinear points
// This is expensive and should not be called at runtime.
B2_API bool b2ValidateHull( const b2Hull* hull );

//
// @defgroup distance Distance
// Functions for computing the distance between shapes.
//
// These are advanced functions you can use to perform distance calculations. There
// are functions for computing the closest points between shapes, doing linear shape casts,
// and doing rotational shape casts. The latter is called time of impact (TOI).
//
*/

// Result of computing the distance between two line segments
type b2SegmentDistanceResult struct {
	// The closest point on the first segment
	Closest1 Vec2

	// The closest point on the second segment
	Closest2 Vec2

	// The barycentric coordinate on the first segment
	Fraction1 float32

	// The barycentric coordinate on the second segment
	Fraction2 float32

	// The squared distance between the closest points
	DistanceSquared float32
}

/*
// Compute the distance between two line segments, clamping at the end points if needed.
B2_API b2SegmentDistanceResult b2SegmentDistance( Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2 );
*/
// Used to warm start the GJK simplex. If you call this function multiple times with nearby
// transforms this might improve performance. Otherwise you can zero initialize this.
// The distance cache must be initialized to zero on the first call.
// Users should generally just zero initialize this structure for each call.
type b2SimplexCache struct {
	// The number of stored simplex points
	Count uint16

	// The cached simplex indices on shape A
	IndexA [3]uint8

	// The cached simplex indices on shape B
	IndexB [3]uint8
}

// Input for b2ShapeDistance
type b2DistanceInput struct {
	// The proxy for shape A
	ProxyA ShapeProxy

	// The proxy for shape B
	ProxyB ShapeProxy

	// The world transform for shape A
	TransformA Transform

	// The world transform for shape B
	TransformB Transform

	// Should the proxy radius be considered?
	UseRadii bool
}

// Output for b2ShapeDistance
type b2DistanceOutput struct {
	// Closest point on shapeA
	PointA Vec2

	// Closest point on shapeB
	PointB Vec2

	// Normal vector that points from A to B. Invalid if distance is zero.
	Normal Vec2

	// The final distance, zero if overlapped
	Distance float32

	// Number of GJK iterations used
	Iterations int32

	// The number of simplexes stored in the simplex array
	SimplexCount int32
}

// Simplex vertex for debugging the GJK algorithm
type b2SimplexVertex struct {
	// support point in proxyA
	WA Vec2

	// support point in proxyB
	WB Vec2

	// wB - wA
	W Vec2

	// barycentric coordinate for closest point
	A float32

	// wA index
	IndexA int32

	// wB index
	IndexB int32
}

// Simplex from the GJK algorithm
type b2Simplex struct {
	// vertices
	V1, V2, V3 b2SimplexVertex

	// number of valid vertices
	Count int32
}

/*
// Compute the closest points between two shapes represented as point clouds.
// b2SimplexCache cache is input/output. On the first call set b2SimplexCache.count to zero.
// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
B2_API b2DistanceOutput b2ShapeDistance( const b2DistanceInput* input, b2SimplexCache* cache, b2Simplex* simplexes,
										 int simplexCapacity );

// Input parameters for b2ShapeCast
type b2ShapeCastPairInput struct {
	b2ShapeProxy proxyA;	//< The proxy for shape A
	b2ShapeProxy proxyB;	//< The proxy for shape B
	b2Transform transformA; //< The world transform for shape A
	b2Transform transformB; //< The world transform for shape B
	Vec2 translationB;	//< The translation of shape B
	float32 maxFraction;		//< The fraction of the translation to consider, typically 1
	bool canEncroach;		//< Allows shapes with a radius to move slightly closer if already touching
} b2ShapeCastPairInput;

// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
// Initially touching shapes are treated as a miss.
B2_API b2CastOutput b2ShapeCast( const b2ShapeCastPairInput* input );

// Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the points.
B2_API b2ShapeProxy b2MakeProxy( const Vec2* points, int count, float32 radius );

// Make a proxy with a transform. This is a deep copy of the points.
B2_API b2ShapeProxy b2MakeOffsetProxy( const Vec2* points, int count, float32 radius, Vec2 position, Rot rotation );

// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
// position.
type b2Sweep struct {
	Vec2 localCenter; //< Local center of mass position
	Vec2 c1;			//< Starting center of mass world position
	Vec2 c2;			//< Ending center of mass world position
	Rot q1;			//< Starting world rotation
	Rot q2;			//< Ending world rotation
} b2Sweep;

// Evaluate the transform sweep at a specific time.
B2_API b2Transform b2GetSweepTransform( const b2Sweep* sweep, float32 time );

// Time of impact input
type b2TOIInput struct {
	b2ShapeProxy proxyA; //< The proxy for shape A
	b2ShapeProxy proxyB; //< The proxy for shape B
	b2Sweep sweepA;		 //< The movement of shape A
	b2Sweep sweepB;		 //< The movement of shape B
	float32 maxFraction;	 //< Defines the sweep interval [0, maxFraction]
} b2TOIInput;

// Describes the TOI output
typedef enum b2TOIState
{
	b2_toiStateUnknown,
	b2_toiStateFailed,
	b2_toiStateOverlapped,
	b2_toiStateHit,
	b2_toiStateSeparated
} b2TOIState;

// Time of impact output
type b2TOIOutput struct {
	// The type of result
	b2TOIState state;

	// The hit point
	Vec2 point;

	// The hit normal
	Vec2 normal;

	// The sweep time of the collision
	float32 fraction;
} b2TOIOutput;

// Compute the upper bound on time before two shapes penetrate. Time is represented as
// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
// non-tunneling collisions. If you change the time interval, you should call this function
// again.
B2_API b2TOIOutput b2TimeOfImpact( const b2TOIInput* input );
*/
//
// @defgroup collision Collision
// @brief Functions for colliding pairs of shapes
//

// A manifold point is a contact point belonging to a contact manifold.
// It holds details related to the geometry and dynamics of the contact points.
// Box2D uses speculative collision so some contact points may be separated.
// You may use the totalNormalImpulse to determine if there was an interaction during
// the time step.
type ManifoldPoint struct {
	// Location of the contact point in world space when first clipped. Subject to precision
	// loss at large coordinates. This point lags behind when contact recycling is used.
	// @note Should only be used for debugging. Use anchorA and/or anchorB for game logic.
	ClipPoint Vec2

	// Location of the contact point relative to shapeA's origin in world space.
	// This can be converted to a world point using:
	// Vec2 worldPointA = b2Add(b2Body_GetCenter(myBodyIdA), anchorA);
	// @note When used internally to the Box2D solver, this is relative to the body center of mass.
	AnchorA Vec2

	// Location of the contact point relative to shapeB's origin in world space
	// This can be converted to a world point using:
	// Vec2 worldPointB = b2Add(b2Body_GetCenter(myBodyIdB), anchorB);
	// @note When used internally to the Box2D solver, this is relative to the body center of mass.
	AnchorB Vec2

	// The separation of the contact point, negative if penetrating
	Separation float32

	// Cached separation used for contact recycling
	BaseSeparation float32

	// The impulse along the manifold normal vector.
	NormalImpulse float32

	// The friction impulse
	TangentImpulse float32

	// The total normal impulse applied across sub-stepping and restitution. This is important
	// to identify speculative contact points that had an interaction in the time step.
	// This includes the warm starting impulse, the sub-step delta impulse, and the restitution
	// impulse.
	TotalNormalImpulse float32

	// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	// zero then there was no hit. Negative means shapes are approaching.
	NormalVelocity float32

	// Uniquely identifies a contact point between two shapes
	Id uint16

	// Did this contact point exist the previous step?
	Persisted bool
}

// A contact manifold describes the contact points between colliding shapes.
// @note Box2D uses speculative collision so some contact points may be separated.
type Manifold struct {
	// The unit normal vector in world space, points from shape A to bodyB
	Normal Vec2

	// Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
	RollingImpulse float32

	// The manifold points, up to two are possible in 2D
	Points [2]ManifoldPoint

	// The number of contacts points, will be 0, 1, or 2
	PointCount int
}

/*
// Compute the contact manifold between two circles
B2_API b2Manifold b2CollideCircles( const b2Circle* circleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB );

// Compute the contact manifold between a capsule and circle
B2_API b2Manifold b2CollideCapsuleAndCircle( const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

// Compute the contact manifold between an segment and a circle
B2_API b2Manifold b2CollideSegmentAndCircle( const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

// Compute the contact manifold between a polygon and a circle
B2_API b2Manifold b2CollidePolygonAndCircle( const Polygon* polygonA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

// Compute the contact manifold between a capsule and circle
B2_API b2Manifold b2CollideCapsules( const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB );

// Compute the contact manifold between an segment and a capsule
B2_API b2Manifold b2CollideSegmentAndCapsule( const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
											  b2Transform xfB );

// Compute the contact manifold between a polygon and capsule
B2_API b2Manifold b2CollidePolygonAndCapsule( const Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB,
											  b2Transform xfB );

// Compute the contact manifold between two polygons
B2_API b2Manifold b2CollidePolygons( const Polygon* polygonA, b2Transform xfA, const Polygon* polygonB, b2Transform xfB );

// Compute the contact manifold between an segment and a polygon
B2_API b2Manifold b2CollideSegmentAndPolygon( const b2Segment* segmentA, b2Transform xfA, const Polygon* polygonB,
											  b2Transform xfB );

// Compute the contact manifold between a chain segment and a circle
B2_API b2Manifold b2CollideChainSegmentAndCircle( const b2ChainSegment* segmentA, b2Transform xfA, const b2Circle* circleB,
												  b2Transform xfB );

// Compute the contact manifold between a chain segment and a capsule
B2_API b2Manifold b2CollideChainSegmentAndCapsule( const b2ChainSegment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
												   b2Transform xfB, b2SimplexCache* cache );

// Compute the contact manifold between a chain segment and a rounded polygon
B2_API b2Manifold b2CollideChainSegmentAndPolygon( const b2ChainSegment* segmentA, b2Transform xfA, const Polygon* polygonB,
												   b2Transform xfB, b2SimplexCache* cache );
*/
//
// @defgroup tree Dynamic Tree
// The dynamic tree is a binary AABB tree to organize and query large numbers of geometric objects
//
// Box2D uses the dynamic tree internally to sort collision shapes into a binary bounding volume hierarchy.
// This data structure may have uses in games for organizing other geometry data and may be used independently
// of Box2D rigid body simulation.
//
// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
// A dynamic tree arranges data in a binary tree to accelerate
// queries such as AABB queries and ray casts. Leaf nodes are proxies
// with an AABB. These are used to hold a user collision object.
// Nodes are pooled and relocatable, so I use node indices rather than pointers.
// The dynamic tree is made available for advanced users that would like to use it to organize
// spatial game data besides rigid bodies.
//

// The dynamic tree structure. This should be considered private data.
// It is placed here for performance reasons.
type DynamicTree struct {
	// The tree nodes
	nodes any //*TreeNode

	// The root index
	Root int32

	// The number of nodes
	NodeCount int32

	// The allocated node space
	NodeCapacity int32

	// Node free list
	FreeList int32

	// Number of proxies created
	ProxyCount int32

	// Leaf indices for rebuild
	LeafIndices *int32

	// Leaf bounding boxes for rebuild
	LeafBoxes *AABB

	// Leaf bounding box centers for rebuild
	LeafCenters *Vec2

	// Bins for sorting during rebuild
	BinIndices *int32

	// Allocated space for rebuilding
	RebuildCapacity *int32
}

// These are performance results returned by dynamic tree queries.
type TreeStats struct {
	// Number of internal nodes visited during the query
	NodeVisits int32

	// Number of leaf nodes visited during the query
	LeafVisits int32
}

// Constructing the tree initializes the node pool.
func DynamicTree_Create(proxyCapacity int) DynamicTree {
	r := C.b2DynamicTree_Create(C.int(proxyCapacity))
	return *cast[DynamicTree](&r)
}

// Destroy the tree, freeing the node pool.
func DynamicTree_Destroy(tree *DynamicTree) {
	C.b2DynamicTree_Destroy(cast[C.b2DynamicTree](tree))
}

// Create a proxy. Provide an AABB and a userData value.
func DynamicTree_CreateProxy(tree *DynamicTree, aabb AABB, categoryBits uint64, userData uint64) int {
	return int(C.b2DynamicTree_CreateProxy(cast[C.b2DynamicTree](tree), cast[C.b2AABB](&aabb), C.uint64_t(categoryBits), C.uint64_t(userData)))
}

// Destroy a proxy. This asserts if the id is invalid.
func DynamicTree_DestroyProxy(tree *DynamicTree, proxyId int) {
	C.b2DynamicTree_DestroyProxy(cast[C.b2DynamicTree](tree), C.int(proxyId))
}

// Move a proxy to a new AABB by removing and reinserting into the tree.
func DynamicTree_MoveProxy(tree *DynamicTree, proxyId int, aabb AABB) {
	C.b2DynamicTree_MoveProxy(cast[C.b2DynamicTree](tree), C.int(proxyId), cast[C.b2AABB](&aabb))
}

// Enlarge a proxy and enlarge ancestors as necessary.
func DynamicTree_EnlargeProxy(tree *DynamicTree, proxyId int, aabb AABB) {
	C.b2DynamicTree_EnlargeProxy(cast[C.b2DynamicTree](tree), C.int(proxyId), cast[C.b2AABB](&aabb))
}

// Modify the category bits on a proxy. This is an expensive operation.
func DynamicTree_SetCategoryBits(tree *DynamicTree, proxyId int, categoryBits uint64) {
	C.b2DynamicTree_SetCategoryBits(cast[C.b2DynamicTree](tree), C.int(proxyId), C.uint64_t(categoryBits))
}

// Get the category bits on a proxy.
func DynamicTree_GetCategoryBits(tree *DynamicTree, proxyId int) uint64 {
	return uint64(C.b2DynamicTree_GetCategoryBits(cast[C.b2DynamicTree](tree), C.int(proxyId)))
}

// This function receives proxies found in the AABB query.
// @return true if the query should continue
type b2TreeQueryCallbackFcn = func(proxyId int, userData uint64, context any) bool

/*
// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
//	@return performance data
func DynamicTree_Query(tree *DynamicTree, aabb AABB, maskBits uint64, callback b2TreeQueryCallbackFcn, context any) TreeStats {

}

// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
// No filtering is performed.
//	@return performance data
func DynamicTree_QueryAll(tree *DynamicTree, aabb AABB, callback b2TreeQueryCallbackFcn, context any) TreeStats {

}
*/
// This function receives clipped ray cast input for a proxy. The function
// returns the new ray fraction.
// - return a value of 0 to terminate the ray cast
// - return a value less than input->maxFraction to clip the ray
// - return a value of input->maxFraction to continue the ray cast without clipping
type b2TreeRayCastCallbackFcn func(input *RayCastInput, proxyId int, userData uint64, context any) float32

// Ray cast against the proxies in the tree. This relies on the callback
// to perform a exact ray cast in the case were the proxy contains a shape.
// The callback also performs the any collision filtering. This has performance
// roughly equal to k * log(n), where k is the number of collisions and n is the
// number of proxies in the tree.
// Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
//	However, this filtering may be approximate, so the user should still apply filtering to results.
// @param tree the dynamic tree to ray cast
// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
// @param maskBits mask bit hint: `bool accept = (maskBits & node->categoryBits) != 0;`
// @param callback a callback class that is called for each proxy that is hit by the ray
// @param context user context that is passed to the callback
//	@return performance data
/*
func DynamicTree_RayCast(tree *DynamicTree, input *RayCastInput, maskBits uint64, callback b2TreeRayCastCallbackFcn, context any) TreeStats {

}
*/

// This function receives clipped ray cast input for a proxy. The function
// returns the new ray fraction.
// - return a value of 0 to terminate the ray cast
// - return a value less than input->maxFraction to clip the ray
// - return a value of input->maxFraction to continue the ray cast without clipping
type b2TreeShapeCastCallbackFcn = func(input *ShapeCastInput, proxyId int, userData uint64, context any) float32

// Ray cast against the proxies in the tree. This relies on the callback
// to perform a exact ray cast in the case were the proxy contains a shape.
// The callback also performs the any collision filtering. This has performance
// roughly equal to k * log(n), where k is the number of collisions and n is the
// number of proxies in the tree.
// @param tree the dynamic tree to ray cast
// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
// @param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
// @param callback a callback class that is called for each proxy that is hit by the shape
// @param context user context that is passed to the callback
//	@return performance data
/*
func DynamicTree_ShapeCast(tree *DynamicTree, input *ShapeCastInput, maskBits uint64, callback b2TreeShapeCastCallbackFcn, context any) TreeStats {

}
*/
// Get the height of the binary tree.
func DynamicTree_GetHeight(tree *DynamicTree) int {
	return int(C.b2DynamicTree_GetHeight(cast[C.b2DynamicTree](tree)))
}

// Get the ratio of the sum of the node areas to the root area.
func DynamicTree_GetAreaRatio(tree *DynamicTree) float32 {
	return float32(C.b2DynamicTree_GetAreaRatio(cast[C.b2DynamicTree](tree)))
}

// Get the bounding box that contains the entire tree
func DynamicTree_GetRootBounds(tree *DynamicTree) AABB {
	r := C.b2DynamicTree_GetRootBounds(cast[C.b2DynamicTree](tree))
	return *cast[AABB](&r)
}

// Get the number of proxies created
func DynamicTree_GetProxyCount(tree *DynamicTree) int {
	return int(C.b2DynamicTree_GetProxyCount(cast[C.b2DynamicTree](tree)))
}

// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
func DynamicTree_Rebuild(tree *DynamicTree, fullBuild bool) int {
	return int(C.b2DynamicTree_Rebuild(cast[C.b2DynamicTree](tree), C.bool(fullBuild)))
}

// Get the number of bytes used by this tree
func DynamicTree_GetByteCount(tree *DynamicTree) int {
	return int(C.b2DynamicTree_GetByteCount(cast[C.b2DynamicTree](tree)))
}

/*
// Get proxy user data
func DynamicTree_GetUserData(tree *DynamicTree, proxyId int) uint64 {

}
*/

// Get the AABB of a proxy
func DynamicTree_GetAABB(tree *DynamicTree, proxyId int) AABB {
	r := C.b2DynamicTree_GetAABB(cast[C.b2DynamicTree](tree), C.int(proxyId))
	return *cast[AABB](&r)
}

// Validate this tree. For testing.
func DynamicTree_Validate(tree *DynamicTree) {
	C.b2DynamicTree_Validate(cast[C.b2DynamicTree](tree))
}

// Validate this tree has no enlarged AABBs. For testing.
func DynamicTree_ValidateNoEnlarged(tree *DynamicTree) {
	C.b2DynamicTree_ValidateNoEnlarged(cast[C.b2DynamicTree](tree))
}

//
// @defgroup character Character mover
// Character movement solver
//

// These are the collision planes returned from b2World_CollideMover
type PlaneResult struct {
	// The collision plane between the mover and a convex shape
	Plane Plane

	// The collision point on the shape.
	Point Vec2

	// Did the collision register a hit? If not this plane should be ignored.
	Hit bool
}

// These are collision planes that can be fed to b2SolvePlanes. Normally
// this is assembled by the user from plane results in b2PlaneResult
type CollisionPlane struct {
	// The collision plane between the mover and some shape
	Plane Plane

	// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can
	// make the plane collision soft. Usually in meters.
	PushLimit float32

	// The push on the mover determined by b2SolvePlanes. Usually in meters.
	Push float32

	// Indicates if b2ClipVector should clip against this plane. Should be false for soft collision.
	ClipVelocity bool
}

// Result returned by b2SolvePlanes
type PlaneSolverResult struct {
	// The translation of the mover
	Translation Vec2

	// The number of iterations used by the plane solver. For diagnostics.
	IterationCount int
}

// Solves the position of a mover that satisfies the given collision planes.
// @param targetDelta the desired movement from the position used to generate the collision planes
// @param planes the collision planes
// @param count the number of collision planes
func b2SolvePlanes(targetDelta Vec2, planes *CollisionPlane, count int) PlaneSolverResult {
	r := C.b2SolvePlanes(cast[C.b2Vec2](&targetDelta), cast[C.b2CollisionPlane](planes), C.int(count))
	return *cast[PlaneSolverResult](&r)
}

// Clips the velocity against the given collision planes. Planes with zero push or clipVelocity
// set to false are skipped.
func b2ClipVector(vector Vec2, planes *CollisionPlane, count int) Vec2 {
	r := C.b2ClipVector(cast[C.b2Vec2](&vector), cast[C.b2CollisionPlane](planes), C.int(count))
	return *cast[Vec2](&r)
}
