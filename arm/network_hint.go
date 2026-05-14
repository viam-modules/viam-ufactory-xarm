package arm

import (
	"fmt"
	"net"
	"runtime"

	"go.viam.com/rdk/logging"
)

// warnIfArmSubnetUnreachable logs a setup hint if no local network interface
// has an address on the same /24 as armHost. Linux only, since the script it
// points at is Linux-specific. Silent on macOS/Windows and on hostname (non-IP)
// hosts, to avoid false positives.
func warnIfArmSubnetUnreachable(logger logging.Logger, armHost string) {
	if runtime.GOOS != "linux" {
		return
	}
	armIP4 := net.ParseIP(armHost).To4()
	if armIP4 == nil {
		return
	}
	ifaces, err := net.Interfaces()
	if err != nil {
		return
	}
	for _, iface := range ifaces {
		addrs, aerr := iface.Addrs()
		if aerr != nil {
			continue
		}
		for _, addr := range addrs {
			ipNet, ok := addr.(*net.IPNet)
			if !ok {
				continue
			}
			ip4 := ipNet.IP.To4()
			if ip4 == nil {
				continue
			}
			if ip4[0] == armIP4[0] && ip4[1] == armIP4[1] && ip4[2] == armIP4[2] {
				return
			}
		}
	}
	subnet := fmt.Sprintf("%d.%d.%d.0/24", armIP4[0], armIP4[1], armIP4[2])
	logger.Warnf(
		"host has no network interface on the arm's subnet %s (arm host %s); "+
			"the module will fail to connect until the host NIC has a static IP on that subnet. "+
			"Run on this host to configure a persistent static IP:\n"+
			"    curl -fsSL https://raw.githubusercontent.com/viam-modules/viam-ufactory-xarm/main/tools/setup-arm-link.sh | sudo bash\n"+
			"See: https://github.com/viam-modules/viam-ufactory-xarm#connecting-using-linux",
		subnet, armHost,
	)
}
