package box2d

/*
#cgo CFLAGS: -I${SRCDIR}/../external/box2d/src -I${SRCDIR}/../external/box2d/include
#cgo android CFLAGS: -DPLATFORM_ANDROID -DPLATFORM_ANDROID_NOMAIN -DGRAPHICS_API_OPENGL_ES2

#include <box2d.h>
#include <stdlib.h>
*/
import "C"
