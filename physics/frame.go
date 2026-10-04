package physics

import (
	"sync"
	"sync/atomic"
)

// bodySnapshot is an immutable copy of a body's observable state.
type bodySnapshot struct {
	id          BodyID
	kind        BodyKind
	pos         Vec2
	angle       float64
	layer       uint32
	mask        uint32
	friction    float64
	restitution float64
	shapes      []Shape // shape values are immutable after AddBody
	aabb        AABB
}

// frameView is an immutable, self-contained view of one completed frame.
// Queries only ever read a frameView, so they can never observe a
// half-stepped world and never block the writer.
type frameView struct {
	frame    uint64
	time     float64
	bodies   []bodySnapshot // sorted by ID
	contacts []ContactDebug // plain data, no pointers into the live world

	// qmu guards rays: queries issued against this frame record
	// themselves here so DebugFrame can report them.
	qmu  sync.Mutex
	rays []RayDebug
}

// recordRay appends a ray query outcome to this frame's query log.
func (fv *frameView) recordRay(r RayDebug) {
	fv.qmu.Lock()
	fv.rays = append(fv.rays, r)
	fv.qmu.Unlock()
}

// rayLog returns a copy of the queries recorded against this frame.
func (fv *frameView) rayLog() []RayDebug {
	fv.qmu.Lock()
	defer fv.qmu.Unlock()
	out := make([]RayDebug, len(fv.rays))
	copy(out, fv.rays)
	return out
}

// atomicFrameView wraps atomic.Pointer for the published frame.
type atomicFrameView struct {
	p atomic.Pointer[frameView]
}

func (a *atomicFrameView) Load() *frameView   { return a.p.Load() }
func (a *atomicFrameView) Store(v *frameView) { a.p.Store(v) }
