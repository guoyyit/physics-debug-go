package physics

import (
	"fmt"
	"testing"
)

func mustNewWorld(t *testing.T, fixedDt float64) *World {
	t.Helper()
	w, err := NewWorld(fixedDt)
	if err != nil {
		t.Fatalf("NewWorld: %v", err)
	}
	return w
}

func mustAdd(t *testing.T, w *World, b Body) {
	t.Helper()
	if err := w.AddBody(b); err != nil {
		t.Fatalf("AddBody(%q): %v", b.ID, err)
	}
}

// scriptedWorld builds a small deterministic scene: a static ground and a
// few dynamic bodies with distinct layers.
func scriptedWorld(t *testing.T) *World {
	t.Helper()
	w := mustNewWorld(t, 1.0/60.0)
	w.SetGravity(Vec2{0, -9.8})
	mustAdd(t, w, Body{ID: "ground", Type: BodyStatic, Layer: 1, Pos: Vec2{0, -5}, Shape: Box{HalfW: 20, HalfH: 1}})
	for i := 0; i < 6; i++ {
		id := fmt.Sprintf("ball-%02d", i)
		mustAdd(t, w, Body{
			ID:    id,
			Type:  BodyDynamic,
			Layer: uint32(1 << (i % 3)),
			Pos:   Vec2{float64(i)*1.5 - 4, float64(i)*1.8 + 2},
			Vel:   Vec2{0.3, 0},
			Shape: Circle{Radius: 0.4 + float64(i)*0.05},
		})
	}
	mustAdd(t, w, Body{ID: "crate", Type: BodyDynamic, Layer: 4, Pos: Vec2{0, 14}, Shape: Box{HalfW: 0.6, HalfH: 0.6}})
	return w
}

// rayScene builds a scene laid out for ray/area query tests.
func rayScene(t *testing.T) *World {
	t.Helper()
	w := mustNewWorld(t, 1.0/60.0)
	for i, id := range []string{"c1", "c2", "c3"} {
		mustAdd(t, w, Body{
			ID:    id,
			Type:  BodyStatic,
			Layer: uint32(1 << i),
			Pos:   Vec2{5 + float64(i)*5, 0},
			Shape: Circle{Radius: 1},
		})
	}
	mustAdd(t, w, Body{ID: "b1", Type: BodyStatic, Layer: 8, Pos: Vec2{5, 4}, Shape: Box{HalfW: 1, HalfH: 1}})
	return w
}
