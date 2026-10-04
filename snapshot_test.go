package physics

import (
	"encoding/json"
	"testing"
)

func TestSnapshotRestoreFrameIdentical(t *testing.T) {
	w := scriptedWorld(t)
	for i := 0; i < 10; i++ {
		w.Step()
	}

	snap := w.Snapshot()
	// Round-trip through JSON to prove the snapshot is process-independent.
	data, err := json.Marshal(snap)
	if err != nil {
		t.Fatalf("marshal snapshot: %v", err)
	}
	var restored WorldSnapshot
	if err := json.Unmarshal(data, &restored); err != nil {
		t.Fatalf("unmarshal snapshot: %v", err)
	}

	w2, err := NewWorldFromSnapshot(&restored)
	if err != nil {
		t.Fatalf("NewWorldFromSnapshot: %v", err)
	}

	// Both worlds continue from frame 10 with no new inputs.
	for i := 0; i < 20; i++ {
		w.Step()
		w2.Step()
		j1, _ := w.DebugJSON()
		j2, _ := w2.DebugJSON()
		if string(j1) != string(j2) {
			t.Fatalf("frame %d diverged after snapshot restore", w.FrameNumber())
		}
	}
}

func TestSnapshotIncludesState(t *testing.T) {
	w := mustNewWorld(t, 1.0/60.0)
	w.SetGravity(Vec2{1.5, -2.5})
	mustAdd(t, w, Body{ID: "a", Type: BodyDynamic, Layer: 3, Pos: Vec2{1, 2}, Rot: 0.7, Vel: Vec2{3, 4}, AngVel: 0.9, Shape: Circle{Radius: 2}})
	if err := w.Step(); err != nil {
		t.Fatal(err)
	}
	if err := w.Advance(0.001); err != nil {
		t.Fatal(err)
	}
	w.SetPaused(true)

	snap := w.Snapshot()
	if snap.Frame != 1 {
		t.Errorf("expected frame 1, got %d", snap.Frame)
	}
	if snap.GravityX != 1.5 || snap.GravityY != -2.5 {
		t.Errorf("gravity not preserved: %+v,%+v", snap.GravityX, snap.GravityY)
	}
	if !snap.Paused {
		t.Error("paused flag not preserved")
	}
	if len(snap.Bodies) != 1 {
		t.Fatalf("expected 1 body, got %d", len(snap.Bodies))
	}
	// Body snapshot must match the post-step state exactly.
	cur := w.Bodies()[0]
	b := snap.Bodies[0]
	if b.ID != "a" || b.Layer != 3 ||
		b.PosX != cur.Pos.X || b.PosY != cur.Pos.Y ||
		b.Rot != cur.Rot || b.VelX != cur.Vel.X || b.VelY != cur.Vel.Y ||
		b.AngVel != cur.AngVel || b.Shape.Radius != 2 {
		t.Errorf("body snapshot mismatch: %+v", b)
	}
	if snap.Accum != 0.001 {
		t.Errorf("accumulator not preserved: %v", snap.Accum)
	}

	w2, err := NewWorldFromSnapshot(snap)
	if err != nil {
		t.Fatal(err)
	}
	if w2.FrameNumber() != 1 {
		t.Errorf("restored frame %d, expected 1", w2.FrameNumber())
	}
	if !w2.Paused() {
		t.Error("restored paused flag is false")
	}
	if w2.Gravity() != (Vec2{1.5, -2.5}) {
		t.Errorf("restored gravity %+v", w2.Gravity())
	}
}

func TestSnapshotIsIndependentOfSource(t *testing.T) {
	w := scriptedWorld(t)
	for i := 0; i < 5; i++ {
		w.Step()
	}
	snap := w.Snapshot()
	w2, err := NewWorldFromSnapshot(snap)
	if err != nil {
		t.Fatal(err)
	}

	// Mutate the source after the snapshot; the restored world must be
	// unaffected.
	w.RemoveBody("ball-00")
	for i := 0; i < 10; i++ {
		w.Step()
	}
	if len(w2.Bodies()) != 8 {
		t.Errorf("restored world affected by source mutation: %d bodies", len(w2.Bodies()))
	}
}

func TestSnapshotRejectsInvalid(t *testing.T) {
	if _, err := NewWorldFromSnapshot(nil); err == nil {
		t.Error("expected error for nil snapshot")
	}
	if _, err := NewWorldFromSnapshot(&WorldSnapshot{Version: 99, FixedDt: 1}); err == nil {
		t.Error("expected error for unsupported version")
	}
	if _, err := NewWorldFromSnapshot(&WorldSnapshot{Version: 1, FixedDt: 0}); err == nil {
		t.Error("expected error for zero fixed dt")
	}
	dup := &WorldSnapshot{
		Version: SnapshotVersion, FixedDt: 1.0 / 60.0,
		Bodies: []BodySnapshot{
			{ID: "a", Type: BodyDynamic, Shape: ShapeDesc{Kind: ShapeCircle, Radius: 1}},
			{ID: "a", Type: BodyDynamic, Shape: ShapeDesc{Kind: ShapeCircle, Radius: 1}},
		},
	}
	if _, err := NewWorldFromSnapshot(dup); err == nil {
		t.Error("expected error for duplicate IDs in snapshot")
	}
	empty := &WorldSnapshot{
		Version: SnapshotVersion, FixedDt: 1.0 / 60.0,
		Bodies: []BodySnapshot{
			{ID: "", Type: BodyDynamic, Shape: ShapeDesc{Kind: ShapeCircle, Radius: 1}},
		},
	}
	if _, err := NewWorldFromSnapshot(empty); err == nil {
		t.Error("expected error for empty ID in snapshot")
	}
	deg := &WorldSnapshot{
		Version: SnapshotVersion, FixedDt: 1.0 / 60.0,
		Bodies: []BodySnapshot{
			{ID: "a", Type: BodyDynamic, Shape: ShapeDesc{Kind: ShapeCircle, Radius: 0}},
		},
	}
	if _, err := NewWorldFromSnapshot(deg); err == nil {
		t.Error("expected error for degenerate shape in snapshot")
	}
}

func TestSnapshotDeterministicOrder(t *testing.T) {
	w := scriptedWorld(t)
	s1 := w.Snapshot()
	s2 := w.Snapshot()
	j1, _ := json.Marshal(s1)
	j2, _ := json.Marshal(s2)
	if string(j1) != string(j2) {
		t.Error("snapshot serialization is not stable")
	}
}
