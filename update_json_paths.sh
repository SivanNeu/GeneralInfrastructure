#!/bin/bash

# Script to update paths in system_managerCPP JSON configuration files

set -e  # Exit on error

# Get the script directory and workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SYSTEM_MANAGER_DIR="$SCRIPT_DIR/system_managerCPP"

echo "=========================================="
echo "JSON Path Update Script"
echo "=========================================="
echo "Workspace root: $WORKSPACE_ROOT"
echo "System Manager directory: $SYSTEM_MANAGER_DIR"
echo ""

# Function to update paths in JSON files
update_json_paths() {
    local json_file="$1"
    local new_base_path="$WORKSPACE_ROOT"
    
    if [ ! -f "$json_file" ]; then
        echo "Warning: JSON file not found: $json_file"
        return
    fi
    
    echo "Updating paths in: $json_file"
    
    # Create a backup
    cp "$json_file" "${json_file}.bak"
    
    # Replace /home/valentin/RL with workspace root
    sed -i "s|/home/valentin/RL|$new_base_path|g" "$json_file"
    
    # Replace /home/pi/RL with workspace root
    sed -i "s|/home/pi/RL|$new_base_path|g" "$json_file"
    
    echo "  ✓ Updated paths in $json_file"
}

# List of JSON files to update
JSON_FILES=(
    "$SYSTEM_MANAGER_DIR/rlcontrol.json"
    "$SYSTEM_MANAGER_DIR/pidcontrol.json"
    "$SYSTEM_MANAGER_DIR/velocity_rl_controller_params.json"
)

for json_file in "${JSON_FILES[@]}"; do
    update_json_paths "$json_file"
done

echo ""
echo "=========================================="
echo "JSON path update complete!"
echo "=========================================="
echo ""
echo "Summary:"
echo "  ✓ Updated paths in JSON configuration files"
echo ""
echo "Note: Backup files (.bak) were created for JSON files"
echo ""
