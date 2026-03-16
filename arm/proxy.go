package arm

import (
	"context"
	"fmt"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"time"
)

const defaultStudioPort = 18333

func (x *xArm) startProxy(ctx context.Context) error {
	port := x.conf.StudioProxyPort
	if port == 0 {
		port = defaultStudioPort
	}

	target := &url.URL{
		Scheme: "http",
		Host:   fmt.Sprintf("%s:%d", x.conf.Host, defaultStudioPort),
	}

	proxy := &httputil.ReverseProxy{
		Rewrite: func(r *httputil.ProxyRequest) {
			r.SetURL(target)
			r.Out.Host = target.Host
		},
	}

	x.proxyServer = &http.Server{
		Addr:              fmt.Sprintf("[::]:%d", port), // '[::]' listens on IPv4 and IPv6
		Handler:           proxy,
		ReadHeaderTimeout: 10 * time.Second,
	}

	var lc net.ListenConfig
	ln, err := lc.Listen(ctx, "tcp", x.proxyServer.Addr)
	if err != nil {
		return fmt.Errorf("studio proxy: failed to listen on port %d: %w", port, err)
	}

	x.logger.Infof("UFactory Studio proxy listening on :%d -> %s", port, target.Host)

	go func() {
		if err := x.proxyServer.Serve(ln); err != nil && err != http.ErrServerClosed {
			x.logger.Errorf("studio proxy error: %v", err)
		}
	}()

	return nil
}

func (x *xArm) stopProxy() {
	if x.proxyServer == nil {
		return
	}

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := x.proxyServer.Shutdown(ctx); err != nil {
		x.logger.Warnf("studio proxy shutdown error: %v", err)
	} else {
		x.logger.Info("UFactory Studio proxy stopped")
	}
	x.proxyServer = nil
}
