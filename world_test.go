package physics

import (
	"fmt"
	"runtime"
	"testing"
)

func TestDeterminism(t *testing.T) {
	w1 := scriptedWorld(t)
	w2 := scriptedWorld(t)

	for i := 0; i < 30; i++ {
		if err := w1.Step(); err != nil {
			t.Fatalf("w1 step %d: %v", i, err)
		}
		if err := w2.Step(); err != nil {
			t.Fatalf("w2 step %d: %v", i, err)
		}
		j1, err := w1.DebugJSON()
		if err != nil {
			t.Fatalf("w1 json: %v", err)
		}
		j2, err := w2.DebugJSON()
		if err != nil {
			t.Fatalf("w2 json: %v", err)
		}
		if string(j1) != string(j2) {
			t.Fatalf("frame %d diverged:\n%s\nvs\n%s", i+1, j1, j2)
		}
	}
}

func TestDeterminismWithRuntimeMutations(t *testing.T) {
	w1 := scriptedWorld(t)
	w2 := scriptedWorld(t)

	// Apply the same mutation sequence to both worlds at the same frames.
	mutate := func(w *World, frame int) {
		switch frame {
		case 5:
			w.SetTransform("ball-00", Vec2{0, 0}, 0)
			w.SetVelocity("ball-01", Vec2{2, 2}, 0)
		case 10:
			w.RemoveBody("ball-02")
			w.AddBody(Body{ID: "new-01", Type: BodyDynamic, Layer: 2, Pos: Vec2{3, 3}, Shape: Circle{Radius: 0.3}})
		case 15:
			w.SetShape("ball-03", Box{HalfW: 0.7, HalfH: 0.7})
			w.SetLayer("ball-04", 16)
		}
	}

	for i := 0; i < 25; i++ {
		mutate(w1, i)
		mutate(w2, i)
		w1.Step()
		w2.Step()
		j1, _ := w1.DebugJSON()
		j2, _ := w2.DebugJSON()
		if string(j1) != string(j2) {
			t.Fatalf("frame %d diverged after mutations", i+1)
		}
	}
}

func TestAdvanceAccumulator(t *testing.T) {
	w := scriptedWorld(t)
	// Advance with dt smaller than fixedDt: no step yet.
	if err := w.Advance(0.001); err != nil {
		t.Fatal(err)
	}
	if w.FrameNumber() != 0 {
		t.Errorf("expected frame 0 after tiny advance, got %d", w.FrameNumber())
	}
	// Advance past the fixed dt: exactly one step.
	if err := w.Advance(1.0/60.0 + 0.001); err != nil {
		t.Fatal(err)
	}
	if w.FrameNumber() != 1 {
		t.Errorf("expected frame 1, got %d", w.FrameNumber())
	}
	// Large dt: multiple steps, no drift.
	if err := w.Advance(1.0); err != nil {
		t.Fatal(err)
	}
	// 1.0 / (1/60) = 60 steps plus leftover; total should be 61.
	if w.FrameNumber() != 61 {
		t.Errorf("expected frame 61, got %d", w.FrameNumber())
	}
	if err := w.Advance(-0.01); err == nil {
		t.Error("expected error for negative dt")
	}
}

func TestPauseAndStep(t *testing.T) {
	w := scriptedWorld(t)
	w.SetPaused(true)
	if !w.Paused() {
		t.Error("expected paused")
	}
	if err := w.Advance(1.0); err != nil {
		t.Fatal(err)
	}
	if w.FrameNumber() != 0 {
		t.Errorf("paused world advanced to frame %d", w.FrameNumber())
	}
	// Single-step works while paused.
	if err := w.Step(); err != nil {
		t.Fatal(err)
	}
	if w.FrameNumber() != 1 {
		t.Errorf("expected single step to frame 1, got %d", w.FrameNumber())
	}
	// Resume: Advance progresses again.
	w.SetPaused(false)
	if err := w.Advance(1.0 / 60.0); err != nil {
		t.Fatal(err)
	}
	if w.FrameNumber() != 2 {
		t.Errorf("expected frame 2 after resume, got %d", w.FrameNumber())
	}
}

func TestAddRemoveUpdateValidation(t *testing.T) {
	w := mustNewWorld(t, 1.0/60.0)

	if err := w.AddBody(Body{ID: "", Shape: Circle{Radius: 1}}); err == nil {
		t.Error("expected error for empty ID")
	}
	if err := w.AddBody(Body{ID: "a", Shape: Circle{Radius: 0}}); err == nil {
		t.Error("expected error for zero radius")
	}
	if err := w.AddBody(Body{ID: "a", Shape: Box{HalfW: 0, HalfH: 1}}); err == nil {
		t.Error("expected error for zero half width")
	}
	if err := w.AddBody(Body{ID: "a", Type: BodyType(99), Shape: Circle{Radius: 1}}); err == nil {
		t.Error("expected error for unknown body type")
	}
	mustAdd(t, w, Body{ID: "a", Type: BodyDynamic, Shape: Circle{Radius: 1}})
	if err := w.AddBody(Body{ID: "a", Type: BodyDynamic, Shape: Circle{Radius: 1}}); err == nil {
		t.Error("expected error for duplicate ID")
	}
	if err := w.RemoveBody("missing"); err == nil {
		t.Error("expected error for removing missing body")
	}
	if err := w.SetTransform("missing", Vec2{}, 0); err == nil {
		t.Error("expected error for updating missing body")
	}
	if err := w.SetShape("a", Circle{Radius: -1}); err == nil {
		t.Error("expected error for invalid replacement shape")
	}
	if err := w.RemoveBody(""); err == nil {
		t.Error("expected error for removing empty ID")
	}

	// Valid updates succeed.
	if err := w.SetTransform("a", Vec2{1, 2}, 0.5); err != nil {
		t.Fatalf("SetTransform: %v", err)
	}
	if err := w.SetVelocity("a", Vec2{3, 4}, 0.1); err != nil {
		t.Fatalf("SetVelocity: %v", err)
	}
	if err := w.SetLayer("a", 8); err != nil {
		t.Fatalf("SetLayer: %v", err)
	}
	if err := w.SetShape("a", Box{HalfW: 2, HalfH: 2}); err != nil {
		t.Fatalf("SetShape: %v", err)
	}
	if err := w.RemoveBody("a"); err != nil {
		t.Fatalf("RemoveBody: %v", err)
	}
	if len(w.Bodies()) != 0 {
		t.Errorf("expected no bodies, got %d", len(w.Bodies()))
	}
}

func TestRuntimeAddRemoveNoGhosts(t *testing.T) {
	w := mustNewWorld(t, 1.0/60.0)
	for i := 0; i < 10; i++ {
		mustAdd(t, w, Body{ID: fmt.Sprintf("b-%02d", i), Type: BodyDynamic, Pos: Vec2{float64(i), 0}, Shape: Circle{Radius: 0.5}})
	}
	w.Step()
	// Remove half the bodies and verify queries never see them again.
	for i := 0; i < 5; i++ {
		id := fmt.Sprintf("b-%02d", i)
		if err := w.RemoveBody(id); err != nil {
			t.Fatal(err)
		}
	}
	res, err := w.OverlapCircle(Vec2{0, 0}, 100, RayFilter{})
	if err != nil {
		t.Fatal(err)
	}
	if len(res.Hits) != 5 {
		t.Fatalf("expected 5 hits after removals, got %d", len(res.Hits))
	}
	for _, h := range res.Hits {
		if h.BodyID < "b-05" {
			t.Errorf("query saw removed body %s", h.BodyID)
		}
	}
}

func TestNoUnboundedMemory(t *testing.T) {
	w := mustNewWorld(t, 1.0/60.0)
	const n = 80
	for i := 0; i < n; i++ {
		mustAdd(t, w, Body{ID: fmt.Sprintf("dyn-%03d", i), Type: BodyDynamic, Pos: Vec2{float64(i % 20), float64(i / 20)}, Shape: Circle{Radius: 0.4}})
	}
	runtime.GC()
	var before runtime.MemStats
	runtime.ReadMemStats(&before)

	// Churn bodies and queries for many frames.
	for step := 0; step < 800; step++ {
		id := fmt.Sprintf("dyn-%03d", step%n)
		if err := w.RemoveBody(id); err != nil {
			t.Fatal(err)
		}
		if err := w.AddBody(Body{ID: id, Type: BodyDynamic, Pos: Vec2{float64(step % 20), 5}, Shape: Circle{Radius: 0.4}}); err != nil {
			t.Fatal(err)
		}
		if _, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 100, RayFilter{}); err != nil {
			t.Fatal(err)
		}
		if err := w.Step(); err != nil {
			t.Fatal(err)
		}
	}
	runtime.GC()
	var after runtime.MemStats
	runtime.ReadMemStats(&after)

	growth := int64(after.HeapAlloc) - int64(before.HeapAlloc)
	if growth > 4<<20 {
		t.Fatalf("heap grew by %d bytes after 800 churn steps; possible unbounded history", growth)
	}
	// Only the latest frame is retained.
	if w.FrameNumber() != 800 {
		t.Errorf("expected frame 800, got %d", w.FrameNumber())
	}
}

func TestNewWorldValidation(t *testing.T) {
	if _, err := NewWorld(0); err == nil {
		t.Error("expected error for zero fixed dt")
	}
	if _, err := NewWorld(-1); err == nil {
		t.Error("expected error for negative fixed dt")
	}
}
