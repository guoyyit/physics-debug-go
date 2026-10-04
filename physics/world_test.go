package physics

import (
	"bytes"
	"fmt"
	"math"
	"sync"
	"testing"
)

// --- helpers ---------------------------------------------------------

func testConfig() Config {
	return Config{Gravity: Vec2{0, -10}, Dt: 1.0 / 60.0, VelocityIterations: 10}
}

func mustWorld(t *testing.T, cfg Config) *World {
	t.Helper()
	w, err := NewWorld(cfg)
	if err != nil {
		t.Fatalf("NewWorld: %v", err)
	}
	return w
}

func mustAdd(t *testing.T, w *World, def BodyDef) {
	t.Helper()
	if err := w.AddBody(def); err != nil {
		t.Fatalf("AddBody(%d): %v", def.ID, err)
	}
}

// groundDef is a wide static floor at y=0.
func groundDef(id BodyID) BodyDef {
	return BodyDef{
		ID:       id,
		Kind:     Static,
		Pos:      Vec2{0, 0},
		Layer:    1,
		Friction: 0.6,
		Shapes:   []Shape{Box{HalfW: 50, HalfH: 1, Offset: Vec2{0, -1}}},
	}
}

func ballDef(id BodyID, x, y float64) BodyDef {
	return BodyDef{
		ID:          id,
		Kind:        Dynamic,
		Pos:         Vec2{x, y},
		Layer:       2,
		Density:     1,
		Friction:    0.4,
		Restitution: 0.2,
		Shapes:      []Shape{Circle{Radius: 0.5}},
	}
}

// bodyPos extracts a body's position from a snapshot.
func bodyPos(s *Snapshot, id BodyID) Vec2 {
	for _, b := range s.Bodies {
		if b.ID == id {
			return b.Pos
		}
	}
	return Vec2{math.NaN(), math.NaN()}
}

// --- determinism -----------------------------------------------------

// TestDeterminism runs the same scenario twice (with bodies inserted in
// different orders) and requires bit-identical states every frame.
func TestDeterminism(t *testing.T) {
	build := func(order []BodyDef) *World {
		w := mustWorld(t, testConfig())
		for _, d := range order {
			mustAdd(t, w, d)
		}
		return w
	}
	defs := []BodyDef{
		groundDef(1),
		ballDef(2, 0, 5),
		ballDef(3, 0.3, 7),
		ballDef(4, -0.2, 9),
		{ID: 5, Kind: Dynamic, Pos: Vec2{2, 6}, Layer: 2, Density: 1,
			Friction: 0.5, Shapes: []Shape{Box{HalfW: 0.4, HalfH: 0.4, Angle: 0.3}}},
	}
	// Same bodies, different insertion order.
	reversed := make([]BodyDef, len(defs))
	for i, d := range defs {
		reversed[len(defs)-1-i] = d
	}

	w1 := build(defs)
	w2 := build(reversed)
	for f := 0; f < 300; f++ {
		w1.Step()
		w2.Step()
		s1, err1 := w1.SaveSnapshot().Encode()
		s2, err2 := w2.SaveSnapshot().Encode()
		if err1 != nil || err2 != nil {
			t.Fatalf("encode: %v %v", err1, err2)
		}
		if !bytes.Equal(s1, s2) {
			t.Fatalf("frame %d: states diverge\n%s\n%s", f, s1, s2)
		}
	}
}

// TestDebugFrameDeterminism checks that identical runs produce
// bit-identical serialized debug frames.
func TestDebugFrameDeterminism(t *testing.T) {
	run := func() []byte {
		w := mustWorld(t, testConfig())
		mustAdd(t, w, groundDef(1))
		mustAdd(t, w, ballDef(2, 0, 3))
		for i := 0; i < 120; i++ {
			w.Step()
		}
		if _, err := w.Raycast(RayQuery{Origin: Vec2{-5, 1}, Dir: Vec2{1, 0}, MaxDist: 20}); err != nil {
			t.Fatal(err)
		}
		data, err := w.DebugFrame().Encode()
		if err != nil {
			t.Fatal(err)
		}
		return data
	}
	a, b := run(), run()
	if !bytes.Equal(a, b) {
		t.Fatalf("debug frames differ:\n%s\n%s", a, b)
	}
}

// --- queries ---------------------------------------------------------

func queryWorld(t *testing.T) *World {
	w := mustWorld(t, testConfig())
	// Three circles along the +x axis on layers 1, 2, 4.
	for i, layer := range []uint32{1, 2, 4} {
		mustAdd(t, w, BodyDef{
			ID:     BodyID(10 + i),
			Kind:   Static,
			Pos:    Vec2{float64(3 + 3*i), 0},
			Layer:  layer,
			Shapes: []Shape{Circle{Radius: 0.5}},
		})
	}
	return w
}

func TestRaycastSortAndFilter(t *testing.T) {
	w := queryWorld(t)
	q := RayQuery{Origin: Vec2{0, 0}, Dir: Vec2{1, 0}, MaxDist: 50}

	res, err := w.Raycast(q)
	if err != nil {
		t.Fatal(err)
	}
	if res.Status != QueryHit || len(res.Hits) != 3 {
		t.Fatalf("want 3 hits, got %v %+v", res.Status, res.Hits)
	}
	for i := 1; i < len(res.Hits); i++ {
		if res.Hits[i-1].Dist >= res.Hits[i].Dist {
			t.Fatalf("hits not sorted by distance: %+v", res.Hits)
		}
	}
	if res.Hits[0].BodyID != 10 || res.Hits[1].BodyID != 11 || res.Hits[2].BodyID != 12 {
		t.Fatalf("unexpected hit order: %+v", res.Hits)
	}

	// Layer filter: only layer 2 (body 11).
	q.Filter.LayerMask = 2
	res, _ = w.Raycast(q)
	if res.Status != QueryHit || len(res.Hits) != 1 || res.Hits[0].BodyID != 11 {
		t.Fatalf("layer filter failed: %v %+v", res.Status, res.Hits)
	}

	// Exclude everything that is hit -> filtered status.
	q.Filter = QueryFilter{Exclude: []BodyID{10, 11, 12}}
	res, _ = w.Raycast(q)
	if res.Status != QueryFiltered {
		t.Fatalf("want filtered, got %v", res.Status)
	}

	// Miss entirely.
	q = RayQuery{Origin: Vec2{0, 10}, Dir: Vec2{1, 0}, MaxDist: 50}
	res, _ = w.Raycast(q)
	if res.Status != QueryNoHit {
		t.Fatalf("want no-hit, got %v", res.Status)
	}
}

func TestRaycastInvalidParams(t *testing.T) {
	w := queryWorld(t)
	cases := []RayQuery{
		{Origin: Vec2{0, 0}, Dir: Vec2{0, 0}, MaxDist: 10},           // zero dir
		{Origin: Vec2{0, 0}, Dir: Vec2{1, 0}, MaxDist: -1},           // negative dist
		{Origin: Vec2{0, 0}, Dir: Vec2{1, 0}, MaxDist: math.NaN()},   // NaN dist
		{Origin: Vec2{math.NaN(), 0}, Dir: Vec2{1, 0}, MaxDist: 10},  // NaN origin
		{Origin: Vec2{0, 0}, Dir: Vec2{math.Inf(1), 0}, MaxDist: 10}, // inf dir
		{Origin: Vec2{0, 0}, Dir: Vec2{1, 0}, MaxDist: math.Inf(1)},  // inf dist
	}
	for i, q := range cases {
		res, err := w.Raycast(q)
		if err == nil {
			t.Fatalf("case %d: expected error, got %+v", i, res)
		}
		pe, ok := err.(*Error)
		if !ok || pe.Code != ErrInvalidParam {
			t.Fatalf("case %d: want ErrInvalidParam, got %v", i, err)
		}
	}
}

func TestRegionQuery(t *testing.T) {
	w := queryWorld(t)
	// Box covering bodies 10 and 11 only.
	res, err := w.OverlapRegion(RegionQuery{Box: AABB{Min: Vec2{0, -2}, Max: Vec2{7, 2}}})
	if err != nil {
		t.Fatal(err)
	}
	if res.Status != QueryHit || len(res.Bodies) != 2 || res.Bodies[0] != 10 || res.Bodies[1] != 11 {
		t.Fatalf("region query: %+v", res)
	}

	// Filtered out.
	res, _ = w.OverlapRegion(RegionQuery{
		Box:    AABB{Min: Vec2{0, -2}, Max: Vec2{7, 2}},
		Filter: QueryFilter{LayerMask: 4},
	})
	if res.Status != QueryFiltered {
		t.Fatalf("want filtered, got %v", res.Status)
	}

	// Empty region.
	res, _ = w.OverlapRegion(RegionQuery{Box: AABB{Min: Vec2{100, 100}, Max: Vec2{101, 101}}})
	if res.Status != QueryNoHit {
		t.Fatalf("want no-hit, got %v", res.Status)
	}

	// Invalid box: Min > Max.
	_, err = w.OverlapRegion(RegionQuery{Box: AABB{Min: Vec2{5, 0}, Max: Vec2{1, 1}}})
	if err == nil {
		t.Fatal("expected error for inverted box")
	}
	if pe := err.(*Error); pe.Code != ErrInvalidParam {
		t.Fatalf("want ErrInvalidParam, got %v", err)
	}
}

// --- runtime mutation --------------------------------------------------

func TestRuntimeMutation(t *testing.T) {
	w := mustWorld(t, testConfig())
	mustAdd(t, w, groundDef(1))
	mustAdd(t, w, ballDef(2, 0, 5))
	for i := 0; i < 30; i++ {
		w.Step()
	}

	// Add a body mid-run: it must appear in queries and debug data.
	mustAdd(t, w, ballDef(3, 10, 1))
	res, err := w.Raycast(RayQuery{Origin: Vec2{10, 10}, Dir: Vec2{0, -1}, MaxDist: 50})
	if err != nil || res.Status != QueryHit || res.Hits[0].BodyID != 3 {
		t.Fatalf("added body not found by raycast: %v %+v", err, res)
	}
	if n := len(w.DebugFrame().Bodies); n != 3 {
		t.Fatalf("debug frame has %d bodies, want 3", n)
	}

	// Teleport it: the query must see the new transform immediately.
	if err := w.SetTransform(3, Vec2{20, 1}, 0); err != nil {
		t.Fatal(err)
	}
	res, _ = w.Raycast(RayQuery{Origin: Vec2{10, 10}, Dir: Vec2{0, -1}, MaxDist: 50})
	if res.Status == QueryHit && res.Hits[0].BodyID == 3 {
		t.Fatal("raycast still sees body 3 at old position")
	}
	res, _ = w.Raycast(RayQuery{Origin: Vec2{20, 10}, Dir: Vec2{0, -1}, MaxDist: 50})
	if res.Status != QueryHit || res.Hits[0].BodyID != 3 {
		t.Fatalf("body 3 not found at new position: %+v", res)
	}

	// Remove it: no trace may remain.
	if err := w.RemoveBody(3); err != nil {
		t.Fatal(err)
	}
	res, _ = w.Raycast(RayQuery{Origin: Vec2{20, 10}, Dir: Vec2{0, -1}, MaxDist: 50})
	if res.Status == QueryHit && res.Hits[0].BodyID == 3 {
		t.Fatal("removed body still hit")
	}
	for _, b := range w.DebugFrame().Bodies {
		if b.ID == 3 {
			t.Fatal("removed body still in debug frame")
		}
	}

	// Unknown IDs are errors, not silent no-ops.
	if err := w.RemoveBody(3); err == nil {
		t.Fatal("removing unknown body must fail")
	} else if pe := err.(*Error); pe.Code != ErrBodyNotFound {
		t.Fatalf("want ErrBodyNotFound, got %v", err)
	}
	if err := w.SetTransform(99, Vec2{}, 0); err == nil {
		t.Fatal("SetTransform on unknown body must fail")
	}
}

// TestQueryLogBounded verifies ray logs do not accumulate across frames.
func TestQueryLogBounded(t *testing.T) {
	w := mustWorld(t, testConfig())
	mustAdd(t, w, ballDef(2, 0, 5))
	for i := 0; i < 50; i++ {
		w.Step()
		// Issue several queries per frame.
		for j := 0; j < 5; j++ {
			if _, err := w.Raycast(RayQuery{Origin: Vec2{0, 10}, Dir: Vec2{0, -1}, MaxDist: 30}); err != nil {
				t.Fatal(err)
			}
		}
	}
	// The debug frame of the latest frame may only contain the queries
	// issued against that frame, not all 250.
	if n := len(w.DebugFrame().Rays); n != 5 {
		t.Fatalf("ray log holds %d entries, want 5 (bounded per frame)", n)
	}
}

// --- snapshot ----------------------------------------------------------

func TestSnapshotRestore(t *testing.T) {
	cfg := testConfig()
	build := func() *World {
		w := mustWorld(t, cfg)
		mustAdd(t, w, groundDef(1))
		mustAdd(t, w, ballDef(2, 0, 5))
		mustAdd(t, w, ballDef(3, 0.25, 7))
		mustAdd(t, w, BodyDef{ID: 4, Kind: Dynamic, Pos: Vec2{-1, 8}, Layer: 2,
			Density: 1, Friction: 0.5, Shapes: []Shape{Box{HalfW: 0.3, HalfH: 0.3, Angle: 0.2}}})
		return w
	}

	// Reference run: 60 + 90 uninterrupted steps.
	ref := build()
	for i := 0; i < 150; i++ {
		ref.Step()
	}
	refSnap, _ := ref.SaveSnapshot().Encode()

	// Interrupted run: 60 steps, snapshot -> encode -> decode -> restore
	// into a *fresh* world, then 90 more steps.
	interrupted := build()
	for i := 0; i < 60; i++ {
		interrupted.Step()
	}
	encoded, err := interrupted.SaveSnapshot().Encode()
	if err != nil {
		t.Fatal(err)
	}
	decoded, err := DecodeSnapshot(encoded)
	if err != nil {
		t.Fatal(err)
	}
	restored := mustWorld(t, cfg) // fresh world, no shared state
	if err := restored.RestoreSnapshot(decoded); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 90; i++ {
		restored.Step()
	}
	gotSnap, _ := restored.SaveSnapshot().Encode()
	if !bytes.Equal(refSnap, gotSnap) {
		t.Fatalf("restored run diverges from reference\nref:  %s\ngot: %s", refSnap, gotSnap)
	}
}

func TestSnapshotRestoreInvalid(t *testing.T) {
	w := mustWorld(t, testConfig())
	mustAdd(t, w, ballDef(2, 0, 5))
	w.Step()
	before, _ := w.SaveSnapshot().Encode()

	// Corrupt snapshot: duplicate IDs.
	bad := &Snapshot{Bodies: []BodyState{
		{ID: 1, Kind: Static, Shapes: []ShapeState{{Kind: ShapeCircle, Radius: 1}}},
		{ID: 1, Kind: Static, Shapes: []ShapeState{{Kind: ShapeCircle, Radius: 1}}},
	}}
	if err := w.RestoreSnapshot(bad); err == nil {
		t.Fatal("duplicate IDs must be rejected")
	} else if pe := err.(*Error); pe.Code != ErrBadSnapshot {
		t.Fatalf("want ErrBadSnapshot, got %v", err)
	}

	// Corrupt snapshot: degenerate shape.
	bad2 := &Snapshot{Bodies: []BodyState{
		{ID: 1, Kind: Static, Shapes: []ShapeState{{Kind: ShapeCircle, Radius: 0}}},
	}}
	if err := w.RestoreSnapshot(bad2); err == nil {
		t.Fatal("degenerate shape must be rejected")
	}

	// World must be unchanged after failed restores.
	after, _ := w.SaveSnapshot().Encode()
	if !bytes.Equal(before, after) {
		t.Fatal("world changed after failed restore")
	}

	// Garbage bytes must not decode.
	if _, err := DecodeSnapshot([]byte("{not json")); err == nil {
		t.Fatal("garbage must not decode")
	}
}

// --- pause / step ------------------------------------------------------

func TestPauseAndStepOnce(t *testing.T) {
	w := mustWorld(t, testConfig())
	mustAdd(t, w, ballDef(2, 0, 5))
	w.Step()
	if w.Frame() != 1 {
		t.Fatalf("frame = %d, want 1", w.Frame())
	}
	w.Pause()
	w.Step()
	w.Step()
	if w.Frame() != 1 {
		t.Fatalf("paused world advanced to frame %d", w.Frame())
	}
	w.StepOnce()
	if w.Frame() != 2 {
		t.Fatalf("StepOnce did not advance, frame = %d", w.Frame())
	}
	w.Resume()
	w.Step()
	if w.Frame() != 3 {
		t.Fatalf("resumed world stuck at frame %d", w.Frame())
	}
}

// --- concurrency -------------------------------------------------------

// TestConcurrentQueries hammers the world with queries while stepping.
// Run with -race. Queries must never fail and must always observe a
// complete frame.
func TestConcurrentQueries(t *testing.T) {
	w := mustWorld(t, testConfig())
	mustAdd(t, w, groundDef(1))
	for i := 0; i < 10; i++ {
		mustAdd(t, w, ballDef(BodyID(10+i), float64(i)*1.5, 5+float64(i)))
	}

	var wg sync.WaitGroup
	stop := make(chan struct{})
	errs := make(chan error, 64)
	for g := 0; g < 8; g++ {
		wg.Add(1)
		go func(g int) {
			defer wg.Done()
			for {
				select {
				case <-stop:
					return
				default:
				}
				q := RayQuery{
					Origin:  Vec2{float64(g) * 2, 20},
					Dir:     Vec2{0, -1},
					MaxDist: 50,
				}
				if _, err := w.Raycast(q); err != nil {
					errs <- fmt.Errorf("raycast: %w", err)
					return
				}
				if _, err := w.OverlapRegion(RegionQuery{
					Box: AABB{Min: Vec2{-5, -5}, Max: Vec2{20, 20}},
				}); err != nil {
					errs <- fmt.Errorf("region: %w", err)
					return
				}
				d := w.DebugFrame()
				// Consistency: the frame index never goes backwards is
				// not required across calls, but body data must be
				// internally complete (every body has shapes).
				for _, b := range d.Bodies {
					if len(b.Shapes) == 0 {
						errs <- fmt.Errorf("body %d with no shapes in debug frame", b.ID)
						return
					}
				}
			}
		}(g)
	}

	var added []BodyID
	for i := 0; i < 500; i++ {
		w.Step()
		if i%50 == 0 {
			id := BodyID(100 + i)
			mustAdd(t, w, ballDef(id, 0, 12))
			added = append(added, id)
		}
		if i%70 == 0 && len(added) > 1 {
			if err := w.RemoveBody(added[0]); err != nil {
				t.Fatal(err)
			}
			added = added[1:]
		}
	}
	close(stop)
	wg.Wait()
	select {
	case err := <-errs:
		t.Fatal(err)
	default:
	}
}

// --- invalid input -----------------------------------------------------

func TestInvalidInput(t *testing.T) {
	// Bad configs.
	if _, err := NewWorld(Config{Dt: 0, VelocityIterations: 1}); err == nil {
		t.Fatal("zero Dt must be rejected")
	}
	if _, err := NewWorld(Config{Dt: 1.0 / 60, VelocityIterations: 0}); err == nil {
		t.Fatal("zero iterations must be rejected")
	}

	w := mustWorld(t, testConfig())

	// Duplicate ID.
	mustAdd(t, w, ballDef(2, 0, 5))
	if err := w.AddBody(ballDef(2, 1, 5)); err == nil {
		t.Fatal("duplicate ID must be rejected")
	} else if pe := err.(*Error); pe.Code != ErrDuplicateID {
		t.Fatalf("want ErrDuplicateID, got %v", err)
	}

	// Degenerate shapes, each must name the body and shape.
	degenerate := []BodyDef{
		{ID: 10, Kind: Static, Shapes: []Shape{Circle{Radius: 0}}},
		{ID: 11, Kind: Static, Shapes: []Shape{Circle{Radius: -1}}},
		{ID: 12, Kind: Static, Shapes: []Shape{Circle{Radius: math.NaN()}}},
		{ID: 13, Kind: Static, Shapes: []Shape{Box{HalfW: 0, HalfH: 1}}},
		{ID: 14, Kind: Static, Shapes: []Shape{Box{HalfW: 1, HalfH: -2}}},
		{ID: 15, Kind: Static, Shapes: nil},
		{ID: 16, Kind: Static, Shapes: []Shape{nil}},
	}
	for _, def := range degenerate {
		if err := w.AddBody(def); err == nil {
			t.Fatalf("body %d: degenerate shape accepted", def.ID)
		}
	}

	// Reserved ID.
	if err := w.AddBody(ballDef(0, 0, 0)); err == nil {
		t.Fatal("ID 0 must be rejected")
	}

	// Non-finite transform.
	if err := w.SetTransform(2, Vec2{math.NaN(), 0}, 0); err == nil {
		t.Fatal("NaN position must be rejected")
	}
}

// --- debug data content ------------------------------------------------

func TestDebugFrameContent(t *testing.T) {
	w := mustWorld(t, testConfig())
	mustAdd(t, w, groundDef(1))
	mustAdd(t, w, ballDef(2, 0, 3))
	// Let the ball fall and rest on the ground.
	for i := 0; i < 180; i++ {
		w.Step()
	}
	d := w.DebugFrame()
	if d.Frame != 180 {
		t.Fatalf("debug frame = %d, want 180", d.Frame)
	}
	if len(d.Bodies) != 2 || d.Bodies[0].ID != 1 || d.Bodies[1].ID != 2 {
		t.Fatalf("bodies not sorted by ID: %+v", d.Bodies)
	}
	// The resting ball must have a contact with the ground.
	found := false
	for _, c := range d.Contacts {
		if (c.A == 1 && c.B == 2) || (c.A == 2 && c.B == 1) {
			found = true
			if len(c.Points) == 0 {
				t.Fatal("contact has no manifold points")
			}
			for _, p := range c.Points {
				if !p.Pos.IsFinite() {
					t.Fatalf("contact point not finite: %+v", p)
				}
			}
		}
	}
	if !found {
		t.Fatal("no contact between ball and ground")
	}
	// Ball must be resting near the ground surface (y ~= 0.5).
	if p := bodyPos(w.SaveSnapshot(), 2); math.Abs(p.Y-0.5) > 0.05 {
		t.Fatalf("ball resting at y=%v, want ~0.5", p.Y)
	}
	// AABB sanity: ball AABB centered at body position.
	ball := d.Bodies[1]
	if !ball.AABB.Contains(ball.Pos) {
		t.Fatalf("body AABB %+v does not contain body pos %+v", ball.AABB, ball.Pos)
	}
	// Ray query must be recorded in the frame's debug data.
	if _, err := w.Raycast(RayQuery{Origin: Vec2{0, 5}, Dir: Vec2{0, -1}, MaxDist: 20}); err != nil {
		t.Fatal(err)
	}
	d = w.DebugFrame()
	if len(d.Rays) != 1 || d.Rays[0].Status != QueryHit || len(d.Rays[0].Hits) == 0 {
		t.Fatalf("recorded ray missing or wrong: %+v", d.Rays)
	}
}
