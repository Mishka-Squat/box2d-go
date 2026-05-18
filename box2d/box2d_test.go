package box2d

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestCreateWorld(t *testing.T) {
	worldDef := DefaultWorldDef()
	world := CreateWorld(&worldDef)
	if assert.True(t, world.IsValid()) {
		defer DestroyWorld(world)
	}
}

func TestCreateBody(t *testing.T) {
	worldDef := DefaultWorldDef()
	world := CreateWorld(&worldDef)
	if assert.True(t, world.IsValid()) {
		defer DestroyWorld(world)
	}

	bodyDef := DefaultBodyDef()
	body := world.CreateBody(&bodyDef)
	if assert.True(t, body.IsValid()) {
		defer DestroyBody(body)
	}
}
