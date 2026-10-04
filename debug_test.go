package physics

import (
	"encoding/json"
	"testing"
)

func TestDebugFrameContents(t *testing.T) {
	w := scriptedWorld(t)
	for i := 0; i < 5; i++ {
		w.Step()
	}
	// Record a ray query that actually hits (downward from above).
	if _, err := w.Raycast(Vec2{0, 25}, Vec2{0, -1}, 100, RayFilter{}); err != nil {
		t.Fatal(err)
	}
	if _, err := w.OverlapCircle(Vec2{0, 0}, 100, RayFilter{}); err != nil {
		t.Fatal(err)
	}

	dbg := w.DebugFrame()
	if dbg.Version != DebugFrameVersion {
		t.Errorf("unexpected debug version %d", dbg.Version)
	}
	if dbg.Frame != w.FrameNumber() {
		t.Errorf("debug frame %d != world frame %d", dbg.Frame, w.FrameNumber())
	}
	if len(dbg.Bodies) != 8 {
		t.Errorf("expected 8 bodies, got %d", len(dbg.Bodies))
	}
	// Every body has an AABB enclosing its shape.
	for _, b := range dbg.Bodies {
		if b.AABB.Min.X > b.AABB.Max.X || b.AABB.Min.Y > b.AABB.Max.Y {
			t.Errorf("body %s has invalid AABB: %+v", b.ID, b.AABB)
		}
		if b.Shape.Kind != ShapeCircle && b.Shape.Kind != ShapeBox {
			t.Errorf("body %s has bad shape kind %d", b.ID, b.Shape.Kind)
		}
	}
	// Contacts exist and are sorted.
	for i := 1; i < len(dbg.Contacts); i++ {
		a, b := dbg.Contacts[i-1], dbg.Contacts[i]
		if a.BodyA > b.BodyA || (a.BodyA == b.BodyA && a.BodyB > b.BodyB) {
			t.Errorf("contacts not sorted: %+v then %+v", a, b)
		}
	}
	// Ray query recorded with hits and positions.
	if len(dbg.RayQueries) != 1 {
		t.Fatalf("expected 1 ray query, got %d", len(dbg.RayQueries))
	}
	rq := dbg.RayQueries[0]
	if len(rq.Hits) == 0 {
		t.Error("expected ray hits in debug data")
	}
	for _, h := range rq.Hits {
		if h.Point == (Vec2{}) {
			t.Errorf("hit for %s has no position", h.BodyID)
		}
	}
	// Area query recorded.
	if len(dbg.AreaQueries) != 1 {
		t.Fatalf("expected 1 area query, got %d", len(dbg.AreaQueries))
	}
}

func TestDebugJSONStable(t *testing.T) {
	build := func() *World {
		w := scriptedWorld(t)
		for i := 0; i < 3; i++ {
			w.Step()
		}
		// Issue the same queries in different orders.
		w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 50, RayFilter{})
		w.Raycast(Vec2{0, 1}, Vec2{1, 0}, 50, RayFilter{})
		w.OverlapCircle(Vec2{0, 0}, 50, RayFilter{})
		return w
	}
	j1, err := build().DebugJSON()
	if err != nil {
		t.Fatal(err)
	}
	j2, err := build().DebugJSON()
	if err != nil {
		t.Fatal(err)
	}
	if string(j1) != string(j2) {
		t.Fatalf("debug JSON not stable:\n%s\nvs\n%s", j1, j2)
	}
}

func TestDebugJSONRoundTrip(t *testing.T) {
	w := scriptedWorld(t)
	w.Step()
	w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 50, RayFilter{})
	w.OverlapBox(Vec2{0, 0}, 10, 10, RayFilter{})

	data, err := w.DebugJSON()
	if err != nil {
		t.Fatal(err)
	}
	var dbg DebugFrame
	if err := json.Unmarshal(data, &dbg); err != nil {
		t.Fatalf("debug JSON did not unmarshal: %v", err)
	}
	if dbg.Frame != 1 || len(dbg.Bodies) != 8 {
		t.Errorf("round-tripped debug frame mismatch: frame=%d bodies=%d", dbg.Frame, len(dbg.Bodies))
	}
}

func TestDebugBodiesSorted(t *testing.T) {
	w := scriptedWorld(t)
	w.Step()
	dbg := w.DebugFrame()
	for i := 1; i < len(dbg.Bodies); i++ {
		if dbg.Bodies[i-1].ID >= dbg.Bodies[i].ID {
			t.Errorf("bodies not sorted: %s then %s", dbg.Bodies[i-1].ID, dbg.Bodies[i].ID)
		}
	}
}

func TestDebugFrameMatchesFrameState(t *testing.T) {
	w := scriptedWorld(t)
	for i := 0; i < 7; i++ {
		w.Step()
	}
	dbg := w.DebugFrame()
	bodies := w.Bodies()
	if len(dbg.Bodies) != len(bodies) {
		t.Fatalf("body count mismatch: debug %d, frame %d", len(dbg.Bodies), len(bodies))
	}
	for i := range bodies {
		d := dbg.Bodies[i]
		b := bodies[i]
		if d.ID != b.ID || d.Pos != b.Pos || d.Rot != b.Rot ||
			d.Vel != b.Vel || d.AngVel != b.AngVel || d.Layer != b.Layer || d.Type != b.Type {
			t.Errorf("body %s debug data does not match frame state", b.ID)
		}
	}
	// Contacts in debug match the frame's contacts.
	frame := w.currentFrame()
	if len(dbg.Contacts) != len(frame.Contacts) {
		t.Errorf("contact count mismatch: debug %d, frame %d", len(dbg.Contacts), len(frame.Contacts))
	}
	if len(dbg.Contacts) != len(frame.Contacts) {
		t.Fatalf("contact count mismatch: debug %d, frame %d", len(dbg.Contacts), len(frame.Contacts))
	}
	for i := range frame.Contacts {
		d, c := dbg.Contacts[i], frame.Contacts[i]
		if d.BodyA != c.BodyA || d.BodyB != c.BodyB || d.Normal != c.Normal {
			t.Errorf("contact %d mismatch: debug %+v, frame %+v", i, d, c)
		}
		if len(d.Points) != len(c.Points) {
			t.Errorf("contact %d point count mismatch", i)
			continue
		}
		for j := range c.Points {
			if d.Points[j] != c.Points[j] {
				t.Errorf("contact %d point %d mismatch: debug %+v, frame %+v", i, j, d.Points[j], c.Points[j])
			}
		}
	}
}
