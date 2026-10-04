package physics

import (
	"errors"
	"testing"
)

func TestRaycastHitsSorted(t *testing.T) {
	w := rayScene(t)
	// Ray along +x from the origin: c1 at x=5 (dist 4), c2 at x=10 (dist 9),
	// c3 at x=15 (dist 14). b1 at y=4 is missed.
	res, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 20, RayFilter{})
	if err != nil {
		t.Fatalf("Raycast: %v", err)
	}
	if len(res.Hits) != 3 {
		t.Fatalf("expected 3 hits, got %d", len(res.Hits))
	}
	want := []string{"c1", "c2", "c3"}
	dists := []float64{4, 9, 14}
	for i, id := range want {
		if res.Hits[i].BodyID != id {
			t.Errorf("hit %d: expected %s, got %s", i, id, res.Hits[i].BodyID)
		}
		if res.Hits[i].Dist != dists[i] {
			t.Errorf("hit %s: expected dist %v, got %v", id, dists[i], res.Hits[i].Dist)
		}
	}
	// Sorted by distance then identifier.
	for i := 1; i < len(res.Hits); i++ {
		a, b := res.Hits[i-1], res.Hits[i]
		if a.Dist > b.Dist || (a.Dist == b.Dist && a.BodyID > b.BodyID) {
			t.Errorf("hits not sorted: %+v then %+v", a, b)
		}
	}
	// First hit normal points back toward the ray origin (-x).
	if res.Hits[0].Normal.X > -0.9 || res.Hits[0].Normal.Y != 0 {
		t.Errorf("unexpected normal for c1: %+v", res.Hits[0].Normal)
	}
}

func TestRaycastNoHitVsFiltered(t *testing.T) {
	w := rayScene(t)

	// Ray that misses everything: empty hits, Filtered == 0.
	miss, err := w.Raycast(Vec2{0, 100}, Vec2{1, 0}, 20, RayFilter{})
	if err != nil {
		t.Fatalf("Raycast miss: %v", err)
	}
	if len(miss.Hits) != 0 {
		t.Errorf("expected no hits, got %d", len(miss.Hits))
	}
	if miss.Filtered != 0 {
		t.Errorf("expected Filtered=0 for a miss, got %d", miss.Filtered)
	}

	// Ray that hits but is fully filtered out: empty hits, Filtered > 0.
	filtered, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 20, RayFilter{Mask: 1 << 3})
	if err != nil {
		t.Fatalf("Raycast filtered: %v", err)
	}
	if len(filtered.Hits) != 0 {
		t.Errorf("expected no visible hits, got %d", len(filtered.Hits))
	}
	if filtered.Filtered != 3 {
		t.Errorf("expected Filtered=3, got %d", filtered.Filtered)
	}

	// Layer mask that admits c1 only.
	one, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 20, RayFilter{Mask: 1})
	if err != nil {
		t.Fatalf("Raycast mask: %v", err)
	}
	if len(one.Hits) != 1 || one.Hits[0].BodyID != "c1" {
		t.Errorf("expected only c1, got %+v", one.Hits)
	}
	if one.Filtered != 2 {
		t.Errorf("expected Filtered=2, got %d", one.Filtered)
	}

	// Predicate filter.
	pred, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 20, RayFilter{
		Accept: func(b *Body) bool { return b.ID == "c2" },
	})
	if err != nil {
		t.Fatalf("Raycast predicate: %v", err)
	}
	if len(pred.Hits) != 1 || pred.Hits[0].BodyID != "c2" {
		t.Errorf("expected only c2, got %+v", pred.Hits)
	}
	if pred.Filtered != 2 {
		t.Errorf("expected Filtered=2, got %d", pred.Filtered)
	}
}

func TestRaycastInvalidParams(t *testing.T) {
	w := rayScene(t)

	if _, err := w.Raycast(Vec2{0, 0}, Vec2{}, 10, RayFilter{}); err == nil {
		t.Error("expected error for zero direction")
	} else {
		var e *Error
		if !errors.As(err, &e) || e.Code != ErrInvalidParam {
			t.Errorf("expected ErrInvalidParam, got %v", err)
		}
	}
	if _, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, -1, RayFilter{}); err == nil {
		t.Error("expected error for negative maxDist")
	}
	if _, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 0, RayFilter{}); err == nil {
		t.Error("expected error for zero maxDist")
	}
	if _, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 10, RayFilter{StaticOnly: true, DynamicOnly: true}); err == nil {
		t.Error("expected error for mutually exclusive filters")
	}
}

func TestRaycastMaxDistance(t *testing.T) {
	w := rayScene(t)
	// maxDist=3 stops before c1 (entry at t=4): no hit.
	res, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 3, RayFilter{})
	if err != nil {
		t.Fatalf("Raycast: %v", err)
	}
	if len(res.Hits) != 0 {
		t.Errorf("expected no hits within maxDist, got %+v", res.Hits)
	}
}

func TestOverlapQueries(t *testing.T) {
	w := rayScene(t)

	// Circle centered on c1: c1 overlaps, c2 is too far.
	c, err := w.OverlapCircle(Vec2{5, 0}, 2, RayFilter{})
	if err != nil {
		t.Fatalf("OverlapCircle: %v", err)
	}
	if len(c.Hits) != 1 || c.Hits[0].BodyID != "c1" {
		t.Errorf("circle overlap: expected [c1], got %+v", c.Hits)
	}

	// Box centered at c1: c1 overlaps; b1 at (5,4) does not reach down.
	b, err := w.OverlapBox(Vec2{5, 0}, 0.5, 0.5, RayFilter{})
	if err != nil {
		t.Fatalf("OverlapBox: %v", err)
	}
	if len(b.Hits) != 1 || b.Hits[0].BodyID != "c1" {
		t.Errorf("box overlap: expected [c1], got %+v", b.Hits)
	}

	// Large circle overlaps everything; layer 4 admits only b1.
	all, err := w.OverlapCircle(Vec2{5, 0}, 100, RayFilter{Mask: 1 << 3})
	if err != nil {
		t.Fatalf("OverlapCircle mask: %v", err)
	}
	if len(all.Hits) != 1 || all.Hits[0].BodyID != "b1" {
		t.Errorf("expected [b1], got %+v", all.Hits)
	}
	if all.Filtered != 3 {
		t.Errorf("expected Filtered=3, got %d", all.Filtered)
	}

	// Hits are sorted by identifier.
	large, _ := w.OverlapCircle(Vec2{5, 0}, 100, RayFilter{})
	for i := 1; i < len(large.Hits); i++ {
		if large.Hits[i-1].BodyID >= large.Hits[i].BodyID {
			t.Errorf("area hits not sorted: %+v", large.Hits)
		}
	}
}

func TestOverlapInvalidParams(t *testing.T) {
	w := rayScene(t)
	if _, err := w.OverlapCircle(Vec2{}, 0, RayFilter{}); err == nil {
		t.Error("expected error for zero radius")
	}
	if _, err := w.OverlapCircle(Vec2{}, -1, RayFilter{}); err == nil {
		t.Error("expected error for negative radius")
	}
	if _, err := w.OverlapBox(Vec2{}, 0, 1, RayFilter{}); err == nil {
		t.Error("expected error for zero half width")
	}
	if _, err := w.OverlapBox(Vec2{}, 1, -1, RayFilter{}); err == nil {
		t.Error("expected error for negative half height")
	}
}

func TestQueryFrameNumber(t *testing.T) {
	w := rayScene(t)
	w.Step()
	w.Step()
	res, err := w.Raycast(Vec2{0, 0}, Vec2{1, 0}, 20, RayFilter{})
	if err != nil {
		t.Fatalf("Raycast: %v", err)
	}
	if res.Frame != w.FrameNumber() {
		t.Errorf("query frame %d != world frame %d", res.Frame, w.FrameNumber())
	}
}
