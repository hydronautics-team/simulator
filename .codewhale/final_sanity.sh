#!/bin/bash
# Final sanity: rexrov spawns, thrusters discovered, per-thruster input moves it.
set -u
cd /home/pilushok/dev/simulator
set +u
source install/setup.bash
set -u
cd .codewhale

cat > stage_world.sdf <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="final_check">
    <physics name="default" type="ode"><gravity>0 0 -9.8</gravity></physics>
    <include>
      <uri>model://rexrov</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
EOF

stdbuf -oL ign gazebo -s -r -v 4 stage_world.sdf > final_check.log 2>&1 &
SIM_PID=$!
for i in $(seq 1 20); do
  sleep 2
  if timeout 3 ign service -l 2>/dev/null | grep -q "/world/final_check/create"; then
    break
  fi
done

echo "thruster input topics: $(grep -c "input on" final_check.log)"
echo "allocator config: $(grep -c "RexrovThrusterSystem: model=" final_check.log)"

( timeout 5 ign topic -t /model/rexrov/thrusters/4/input -m ignition.msgs.Double \
    -p 'data: 2000' > /dev/null 2>&1 ) &
PUB_PID=$!
sleep 1
X1=$(timeout 2 ign topic -e -t /world/final_check/pose/info 2>/dev/null \
  | grep -A6 'name: "rexrov"' | grep "x:" | head -1)
sleep 3
X2=$(timeout 2 ign topic -e -t /world/final_check/pose/info 2>/dev/null \
  | grep -A6 'name: "rexrov"' | grep "x:" | head -1)
echo "x before: $X1"
echo "x after:  $X2"
wait "$PUB_PID" 2>/dev/null

kill "$SIM_PID" 2>/dev/null
wait "$SIM_PID" 2>/dev/null
rm -f stage_world.sdf final_check.log
echo DONE
