package box2d

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestWorld(t *testing.T) {
	wd := DefaultWorldDef()
	world := CreateWorld(&wd)
	assert.True(t, world.IsValid())
	DestroyWorld(world)
}
