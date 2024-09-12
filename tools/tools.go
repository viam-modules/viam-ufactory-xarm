//go:build tools
// +build tools

// Package tools defines helper build time tooling needed by the codebase.
package tools

import (
	// for importing tools.
	_ "github.com/AlekSi/gocov-xml"
	_ "github.com/axw/gocov/gocov"
	_ "github.com/edaniels/golinters/cmd/combined"
	_ "github.com/fullstorydev/grpcurl/cmd/grpcurl"
	_ "github.com/golangci/golangci-lint/cmd/golangci-lint"
	_ "github.com/rhysd/actionlint"
	_ "gotest.tools/gotestsum"
)
