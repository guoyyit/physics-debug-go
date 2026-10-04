package physics

import "fmt"

// ErrorCode classifies physics-world errors so callers can distinguish
// invalid input, missing objects and duplicate identifiers.
type ErrorCode int

const (
	// ErrInvalidParam means a numeric/flag argument is out of range or invalid.
	ErrInvalidParam ErrorCode = iota + 1
	// ErrDuplicateID means an identifier was used twice.
	ErrDuplicateID
	// ErrNotFound means a referenced object does not exist in the world.
	ErrNotFound
	// ErrInvalidShape means a shape is degenerate (e.g. zero radius).
	ErrInvalidShape
	// ErrEmptyID means an empty identifier was supplied.
	ErrEmptyID
)

// Error carries a machine-readable code plus human-readable context that
// points at the offending object, parameter and value.
type Error struct {
	Code    ErrorCode
	Object  string
	Param   string
	Value   string
	Message string
}

func (e *Error) Error() string {
	s := "physics: " + e.Message
	if e.Object != "" {
		s += fmt.Sprintf(" [object=%q]", e.Object)
	}
	if e.Param != "" {
		s += fmt.Sprintf(" [param=%s]", e.Param)
	}
	if e.Value != "" {
		s += fmt.Sprintf(" [value=%s]", e.Value)
	}
	return s
}
