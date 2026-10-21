# Pending task routes — do not activate during this implementation

Installed domain_bridge: 0.5.0. Syntax verified against
`/opt/ros/humble/include/domain_bridge/parse_domain_bridge_yaml_config.hpp`.
Five topic routes per PC, reliable/volatile, depth 10 (state/command) or 50 (observations).

| Source | Ranger PC bridge | Go2 PC bridge |
|---|---|---|
| /multi_agent/ranger/state + observation | 0→40 | 40→0 |
| /multi_agent/go2/state + observation + command | 40→0 | 0→40 |

Go2 uses its existing `cyclonedds_go2_multi_agent_bridge.xml`:
Domain0=enp4s0 ONLY; Domain40=wlo1 ONLY. Ranger must provide its own
domain-specific DDS configuration with local Domain0 isolated from Wi-Fi
discovery; do not copy Go2's enp4s0 interface assumption to Ranger.

These pending YAML files carry new multi_agent_task_msgs only. They do not
replace config/multi_agent_bridge.yaml or old /multi_agent/agents/*/state routes.
Never set bidirectional=true on task routes or export imported messages.
No TF/map/odom/scan/Nav2 action routes. No Domain31 bridge.

Install/source identical interfaces in each agent and bridge runtime before
field activation. Custom types, QoS, DDS isolation and loop-free behavior
require subsequent cross-PC acceptance; static parser checks are not communication tests.
