#!/bin/bash
# Cleanup script for drone swarm simulation
# Kills Gazebo, ArduPilot SITL processes and frees associated ports

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== Drone Swarm Cleanup ===${NC}"

# Kill Gazebo processes
echo -e "\n${YELLOW}Killing Gazebo processes...${NC}"
pkill -9 -f "gz sim" 2>/dev/null && echo -e "${GREEN}Killed gz sim${NC}" || echo "No gz sim processes"
pkill -9 -f "ruby.*gz" 2>/dev/null && echo -e "${GREEN}Killed gz ruby processes${NC}" || echo "No gz ruby processes"
pkill -9 -f "gzserver" 2>/dev/null && echo -e "${GREEN}Killed gzserver${NC}" || echo "No gzserver processes"
pkill -9 -f "gzclient" 2>/dev/null && echo -e "${GREEN}Killed gzclient${NC}" || echo "No gzclient processes"

# Kill ArduPilot SITL processes
echo -e "\n${YELLOW}Killing ArduPilot SITL processes...${NC}"
pkill -9 -f "sim_vehicle.py" 2>/dev/null && echo -e "${GREEN}Killed sim_vehicle.py${NC}" || echo "No sim_vehicle.py processes"
pkill -9 -f "arducopter" 2>/dev/null && echo -e "${GREEN}Killed arducopter${NC}" || echo "No arducopter processes"
pkill -9 -f "ardupilot" 2>/dev/null && echo -e "${GREEN}Killed ardupilot${NC}" || echo "No ardupilot processes"

# Kill MAVProxy processes
echo -e "\n${YELLOW}Killing MAVProxy processes...${NC}"
pkill -9 -f "mavproxy" 2>/dev/null && echo -e "${GREEN}Killed mavproxy${NC}" || echo "No mavproxy processes"

# Kill any Python processes related to our swarm scripts
echo -e "\n${YELLOW}Killing swarm-related Python processes...${NC}"
pkill -9 -f "launch_sitl.py" 2>/dev/null && echo -e "${GREEN}Killed launch_sitl.py${NC}" || echo "No launch_sitl.py processes"
pkill -9 -f "fly_swarm" 2>/dev/null && echo -e "${GREEN}Killed fly_swarm scripts${NC}" || echo "No fly_swarm processes"
pkill -9 -f "test_fleet" 2>/dev/null && echo -e "${GREEN}Killed test_fleet${NC}" || echo "No test_fleet processes"

# Free ports used by SITL and MAVSDK
echo -e "\n${YELLOW}Freeing simulation ports...${NC}"

# Function to kill process on a specific port
free_port() {
    local port=$1
    local pid=$(lsof -t -i:$port 2>/dev/null)
    if [ -n "$pid" ]; then
        kill -9 $pid 2>/dev/null && echo -e "${GREEN}Freed port $port (PID: $pid)${NC}"
    fi
}

# SITL TCP ports (5760 + N*10 for instances 0-9)
for i in {0..9}; do
    port=$((5760 + i * 10))
    free_port $port
done

# MAVSDK UDP ports (14540 + N*10 for instances 0-9)
for i in {0..9}; do
    port=$((14540 + i * 10))
    free_port $port
done

# FDM ports (9002 + N*10 for instances 0-9)
for i in {0..9}; do
    port=$((9002 + i * 10))
    free_port $port
    # Also fdm_port_out = fdm_port_in + 3
    free_port $((9002 + i * 10 + 3))
done

# Common ports
free_port 14550  # QGroundControl default
free_port 5762   # SITL console
free_port 5763   # SITL extra

echo -e "\n${GREEN}=== Cleanup Complete ===${NC}"

# Show any remaining simulation processes
remaining=$(pgrep -f "gz|ardupilot|arducopter|mavproxy|sim_vehicle" 2>/dev/null | wc -l)
if [ "$remaining" -gt 0 ]; then
    echo -e "\n${YELLOW}Warning: $remaining potentially related processes still running${NC}"
    echo "Run 'ps aux | grep -E \"gz|ardupilot|arducopter|mavproxy\"' to inspect"
else
    echo -e "\n${GREEN}All simulation processes terminated${NC}"
fi
