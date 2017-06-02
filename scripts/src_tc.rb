#!/usr/bin/env ruby

require 'optparse'

# The physical interface
iface = ARGV[0] 

# Downlink bandwidth limit
downlinkBandwidth = "1mbit"

# Uplink bandwidth limit
uplinkBandwidth = "100mbit"

# Default options
options = {:iface => "eth0", :downlink => "100mb", :uplink => "10mb", :filter => "127.0.0.1", :latency => "0ms"}

OptionParser.new do |opts|
  opts.banner = "Usage: sudo ./src_tc.rb [options]\n\n" +
    "A bandwidth value must have one of the following suffixes:\n" +
    "  b\t\tBytes\n" +
    "  kbit\t\tKilobits\n"+
    "  k, or kb\tKilobytes\n"+
    "  mbit\t\tMegabits\n" +
    "  m, or mb\tMegabits\n" +
    "  gbit\t\tGigabits" +
    "  g, or gb\t Gigabytes\n\n" +
    "Example:\n  sudo ./src_tc.rb -i eth0 -d 100mbit -u 10kb -f 192.168.2.150\n\n" +
    "Options:"

  opts.on('-i', '--iface INTERFACE', "Ethernet interface name. Default=#{options[:iface]}") { |v|
    options[:iface] = v
  }

  opts.on('-d', '--downlink BANDWIDTH', "Downlink bandwidth. Default=#{options[:downlink]}") { |v|
    options[:downlink] = v
  }

  opts.on('-u', '--uplink BANDWIDTH', "Uplink bandwidth. Default=#{options[:uplink]}") { |v|
    options[:uplink] = v
  }

  opts.on('-f', '--filter IP', "IP filter. Default=#{options[:filter]}") { |v|
    options[:filter] = v
  }

  opts.on('-l', '--latency DELAY', "Latency delay applied to uplink and downlink. Default=#{options[:latency]}") { |v|
    options[:latency] = v
  }


end.parse!

# Clear tc
`tc qdisc del dev #{options[:iface]} root`
`tc qdisc del dev ifb0 root`

# Insert the ifb module so that we can redirect incoming (ingress) traffic
# to a virtual interface. This will allow us to apply a bandwidth limit to
# incoming traffic. Bandwidth limits can only be applied to send queues.
# This is why we must redirect incoming traffic to a virtual interface, and
# then limit the virtual interface's outbound queue.
`modprobe ifb numifbs=1`
# Create the virtual interface `ip link set dev ifb0 up` 
# Redirect ingress traffic from the physical interface to the virtual
# interface.
`tc qdisc add dev #{options[:iface]} handle ffff: ingress`
`tc filter add dev #{options[:iface]} parent ffff: protocol ip u32 match u32 0 0 action mirred egress redirect dev ifb0`

# Apply egress (uplink) rules for the physical interface
`tc qdisc add dev #{options[:iface]} root handle 1: htb`
`tc class add dev #{options[:iface]} parent 1: classid 1:10 htb rate #{options[:uplink]}`
`tc filter add dev #{options[:iface]} protocol ip parent 1: prio 1 u32 match ip dst #{options[:filter]} flowid 1:10`
`tc qdisc add dev #{options[:iface]} parent 1:10 netem delay #{options[:latency]}`

# Apply ingress (downlink) rules for the physical interface
# via egress rules for the virtual interface.
`tc qdisc add dev ifb0 root handle 1: htb`
`tc class add dev ifb0 parent 1: classid 1:10 htb rate #{options[:downlink]}`
`tc filter add dev ifb0 protocol ip parent 1: prio 1 u32 match ip src #{options[:filter]} flowid 1:10`
`tc qdisc add dev ifb0 parent 1:10 netem delay #{options[:latency]}`

