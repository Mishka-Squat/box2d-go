package box2d

import (
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
