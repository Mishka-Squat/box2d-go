package box2d

import (
	"fmt"
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestCreateWorld(t *testing.T) {
	worldDef := DefaultWorldDef()
	world := CreateWorld(&worldDef)
	if assert.True(t, world.IsValid()) {
		defer world.Defer()
	}
}

func TestCreateBody(t *testing.T) {
	worldDef := DefaultWorldDef()
	world := CreateWorld(&worldDef)
	if assert.True(t, world.IsValid()) {
		defer world.Defer()
	}

	bodyDef := DefaultBodyDef()
	body := world.CreateBody(&bodyDef)
	if assert.True(t, body.IsValid()) {
		defer body.Defer()
	}
}

func TestSample(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2{
		X: 0, Y: -10,
	}
	world := CreateWorld(&worldDef)
	if assert.True(t, world.IsValid()) {
		defer world.Defer()
	}

	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{
		X: 0, Y: -10,
	}
	ground := world.CreateBody(&groundDef)
	if assert.True(t, ground.IsValid()) {
		defer ground.Defer()
	}

	groundBox := MakeBox(50.0, 10.0)
	groundShapeDef := DefaultShapeDef()

	groundShape := ground.CreatePolygonShape(&groundShapeDef, groundBox)
	assert.True(t, groundShape.IsValid())

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: 0.0, Y: 4.0}
	body := world.CreateBody(&bodyDef)
	assert.True(t, body.IsValid())

	dynamicBox := MakeBox(1.0, 1.0)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.Material.Friction = 0.3

	dynamicShape := body.CreatePolygonShape(&shapeDef, dynamicBox)
	assert.True(t, dynamicShape.IsValid())

	var timeStep float32 = 1.0 / 60.0
	subStepCount := 4
	for range 90 {
		world.Step(timeStep, subStepCount)
		position := body.GetPosition()
		//rotation := body.GetRotation()
		fmt.Printf("%s\n", position /*, b2Rot_GetAngle(rotation)*/)
	}
}

func TestComputeAABB(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2{
		X: 0, Y: -10,
	}
	world := CreateWorld(&worldDef)
	if assert.True(t, world.IsValid()) {
		defer world.Defer()
	}

	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{
		X: 0, Y: -10,
	}
	ground := world.CreateBody(&groundDef)
	if assert.True(t, ground.IsValid()) {
		defer ground.Defer()
	}

	groundBox := MakeBox(50.0, 10.0)
	groundShapeDef := DefaultShapeDef()

	groundShape := ground.CreatePolygonShape(&groundShapeDef, groundBox)
	assert.True(t, groundShape.IsValid())

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: 0.0, Y: 4.0}
	body := world.CreateBody(&bodyDef)
	assert.True(t, body.IsValid())

	dynamicBox := MakeBox(1.0, 1.0)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.Material.Friction = 0.3

	dynamicShape := body.CreatePolygonShape(&shapeDef, dynamicBox)
	assert.True(t, dynamicShape.IsValid())

	aabb := world.ComputeAABB()
	fmt.Printf("%s\n", aabb /*, b2Rot_GetAngle(rotation)*/)
}
