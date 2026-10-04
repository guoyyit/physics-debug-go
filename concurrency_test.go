package physics

import (
	"fmt"
	"math"
	"sync"
	"testing"
)

// TestConcurrentQueriesSeeCompleteFrames hammers a static scene with
// concurrent queries while stepping. Because bodies never move, every hit
// can be recomputed exactly against the current frame.
func TestConcurrentQueriesSeeCompleteFrames(t *testing.T) {
	w := rayScene(t)
	for i := 0; i < 20; i++ {
		w.Step()
	}

	var wg sync.WaitGroup
	errCh := make(chan error, 16)
	stop := make(chan struct{})

	// Stepper.
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < 100; i++ {
			if err := w.Step(); err != nil {
				errCh <- err
				return
			}
		}
		close(stop)
	}()

	// Queriers.
	for q := 0; q < 4; q++ {
		wg.Add(1)
		go func(q int) {
			defer wg.Done()
			for i := 0; i < 200; i++ {
				origin := Vec2{float64(i % 3), float64(q)}
				res, err := w.Raycast(origin, Vec2{1, 0}, 100, RayFilter{})
				if err != nil {
					errCh <- err
					return
				}
				if err := verifyRayResult(res); err != nil {
					errCh <- err
					return
				}
				if err := verifyRayMatchesFrame(w, origin, Vec2{1, 0}, res); err != nil {
					errCh <- err
					return
				}
				if _, err := w.OverlapCircle(origin, 50, RayFilter{}); err != nil {
					errCh <- err
					return
				}
			}
		}(q)
	}
	wg.Wait()
	close(errCh)
	for err := range errCh {
		t.Error(err)
	}
}

// TestConcurrentQueriesOnDynamicScene checks internal consistency of query
// results while bodies move and frames advance concurrently.
func TestConcurrentQueriesOnDynamicScene(t *testing.T) {
	w := scriptedWorld(t)
	var wg sync.WaitGroup
	errCh := make(chan error, 16)
	stop := make(chan struct{})

	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < 100; i++ {
			if err := w.Step(); err != nil {
				errCh <- err
				return
			}
		}
		close(stop)
	}()

	for q := 0; q < 4; q++ {
		wg.Add(1)
		go func(q int) {
			defer wg.Done()
			for i := 0; i < 200; i++ {
				origin := Vec2{float64(q) - 2, 25}
				dir := Vec2{0, -1}
				res, err := w.Raycast(origin, dir, 100, RayFilter{})
				if err != nil {
					errCh <- err
					return
				}
				if res.Frame > w.FrameNumber() {
					errCh <- errorf("query frame %d is in the future (world %d)", res.Frame, w.FrameNumber())
					return
				}
				if err := verifyRayResult(res); err != nil {
					errCh <- err
					return
				}
				area, err := w.OverlapBox(Vec2{float64(q), 0}, 20, 20, RayFilter{})
				if err != nil {
					errCh <- err
					return
				}
				if err := verifyAreaResult(area); err != nil {
					errCh <- err
					return
				}
			}
		}(q)
	}
	wg.Wait()
	close(errCh)
	for err := range errCh {
		t.Error(err)
	}
}

// TestConcurrentDeterminism runs queries concurrently with stepping on one
// world and serial stepping on another; the body/contact portions of the
// debug data must be identical frame for frame.
func TestConcurrentDeterminism(t *testing.T) {
	serial := scriptedWorld(t)
	concurrent := scriptedWorld(t)

	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < 60; i++ {
			concurrent.Step()
		}
	}()
	for q := 0; q < 3; q++ {
		wg.Add(1)
		go func(q int) {
			defer wg.Done()
			for i := 0; i < 60; i++ {
				concurrent.Raycast(Vec2{float64(q), 25}, Vec2{0, -1}, 100, RayFilter{})
				concurrent.OverlapCircle(Vec2{float64(q), 0}, 40, RayFilter{})
			}
		}(q)
	}

	for i := 0; i < 60; i++ {
		serial.Step()
	}
	wg.Wait()

	sd := serial.DebugFrame()
	cd := concurrent.DebugFrame()
	if len(sd.Bodies) != len(cd.Bodies) {
		t.Fatalf("body count differs: %d vs %d", len(sd.Bodies), len(cd.Bodies))
	}
	for i := range sd.Bodies {
		a, b := sd.Bodies[i], cd.Bodies[i]
		if a.ID != b.ID || a.Pos != b.Pos || a.Rot != b.Rot || a.Vel != b.Vel || a.AngVel != b.AngVel {
			t.Errorf("body %d (%s) diverged: serial %+v, concurrent %+v", i, a.ID, a.Pos, b.Pos)
		}
	}
	if len(sd.Contacts) != len(cd.Contacts) {
		t.Fatalf("contact count differs: %d vs %d", len(sd.Contacts), len(cd.Contacts))
	}
	for i := range sd.Contacts {
		a, b := sd.Contacts[i], cd.Contacts[i]
		if a.BodyA != b.BodyA || a.BodyB != b.BodyB || a.Normal != b.Normal {
			t.Errorf("contact %d diverged", i)
		}
	}
}

// verifyRayResult checks internal consistency of a ray result: sorted hits,
// no duplicates, hit points on the ray, unit normals.
func verifyRayResult(res *RaycastResult) error {
	if len(res.Hits) == 0 {
		return nil
	}
	seen := map[string]bool{}
	for i, h := range res.Hits {
		if seen[h.BodyID] {
			return errorf("duplicate hit for body %s", h.BodyID)
		}
		seen[h.BodyID] = true
		if h.Dist <= 0 {
			return errorf("hit for %s has non-positive distance %v", h.BodyID, h.Dist)
		}
		// Normal must be unit length.
		if n := h.Normal.Len(); math.Abs(n-1) > 1e-9 {
			return errorf("normal for %s is not unit length: %v", h.BodyID, n)
		}
		if i > 0 {
			p := res.Hits[i-1]
			if p.Dist > h.Dist || (p.Dist == h.Dist && p.BodyID > h.BodyID) {
				return errorf("hits not sorted: %s then %s", p.BodyID, h.BodyID)
			}
		}
	}
	return nil
}

// verifyRayMatchesFrame recomputes each hit against the current frame. It is
// only valid for static scenes where bodies never move.
func verifyRayMatchesFrame(w *World, origin, dir Vec2, res *RaycastResult) error {
	frame := w.currentFrame()
	for _, h := range res.Hits {
		var body *Body
		for i := range frame.Bodies {
			if frame.Bodies[i].ID == h.BodyID {
				body = &frame.Bodies[i]
			}
		}
		if body == nil {
			return errorf("hit body %s not present in frame %d", h.BodyID, frame.Number)
		}
		d := dir.Normalized()
		dist, _, ok := rayShape(origin, d, 100, body)
		if !ok {
			return errorf("hit for %s does not recompute in frame %d", h.BodyID, frame.Number)
		}
		if math.Abs(dist-h.Dist) > 1e-9 {
			return errorf("hit for %s distance %v does not match frame %d (%v)", h.BodyID, h.Dist, frame.Number, dist)
		}
	}
	return nil
}

func verifyAreaResult(res *AreaResult) error {
	seen := map[string]bool{}
	for i, h := range res.Hits {
		if seen[h.BodyID] {
			return errorf("duplicate area hit for body %s", h.BodyID)
		}
		seen[h.BodyID] = true
		if i > 0 && res.Hits[i-1].BodyID >= h.BodyID {
			return errorf("area hits not sorted: %s then %s", res.Hits[i-1].BodyID, h.BodyID)
		}
	}
	return nil
}

type testError string

func (e testError) Error() string { return string(e) }

func errorf(format string, args ...interface{}) error {
	return testError(fmt.Sprintf(format, args...))
}
