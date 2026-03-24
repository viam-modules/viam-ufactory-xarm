package arm

import (
	"context"
	"net"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

func TestStartProxyPortConflict(t *testing.T) {
	ctx := context.Background()

	// Occupy a port on all interfaces (matching startProxy's "[::]:<port>" bind) and keep it open.
	var lc net.ListenConfig
	ln, err := lc.Listen(ctx, "tcp", "[::]:0")
	test.That(t, err, test.ShouldBeNil)
	port := ln.Addr().(*net.TCPAddr).Port
	defer ln.Close()

	x := &xArm{
		conf: &Config{
			Host:            "127.0.0.1",
			StudioProxy:     true,
			StudioProxyPort: port,
		},
		logger: logging.NewTestLogger(t),
	}

	err = x.startProxy(ctx)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "failed to listen")
}

func TestStopProxyIdempotent(t *testing.T) {
	x := &xArm{
		conf:   &Config{Host: "127.0.0.1"},
		logger: logging.NewTestLogger(t),
	}
	x.stopProxy()
	x.stopProxy()
}
