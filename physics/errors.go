package physics

import "fmt"

// ErrorCode classifies failures so callers can react programmatically.
type ErrorCode int

const (
	// ErrDuplicateID: a body with the same ID already exists.
	ErrDuplicateID ErrorCode = iota + 1
	// ErrBodyNotFound: the referenced body ID does not exist.
	ErrBodyNotFound
	// ErrDegenerateShape: zero/negative size, NaN or otherwise unusable shape.
	ErrDegenerateShape
	// ErrInvalidParam: a query or config parameter is out of range.
	ErrInvalidParam
	// ErrBadSnapshot: snapshot data failed validation during restore.
	ErrBadSnapshot
)

// Error is the single error type returned by this package. It always
// names the operation and the offending object or parameter.
type Error struct {
	Code ErrorCode
	Op   string // e.g. "World.AddBody"
	Msg  string // human-readable detail pointing at the object/parameter
}

func (e *Error) Error() string {
	return fmt.Sprintf("%s: %s", e.Op, e.Msg)
}

func newError(code ErrorCode, op, format string, args ...any) *Error {
	return &Error{Code: code, Op: op, Msg: fmt.Sprintf(format, args...)}
}
