#!/usr/bin/env bash
set -euo pipefail

# ---------- Tunables (override via env) ----------
WORLD="${WORLD:-empty}"
TABLE_TOP_Z="${TABLE_TOP_Z:-0.30}"
TABLE_SIZE_X="${TABLE_SIZE_X:-1.0}"
TABLE_SIZE_Y="${TABLE_SIZE_Y:-1.5}"
TABLE_THICK="${TABLE_THICK:-0.05}"

X="${X:-0.45}"
SPACING="${SPACING:-0.08}"
BLOCK_SIZE="${BLOCK_SIZE:-0.04}"
BLOCK_DROP="${BLOCK_DROP:-0.01}"

# Mass OR density
BLOCK_MASS="${BLOCK_MASS:-0.02}"
BLOCK_DENSITY="${BLOCK_DENSITY:-}"

# friction/contact
BLOCK_MU="${BLOCK_MU:-2.0}"
BLOCK_MU2="${BLOCK_MU2:-2.0}"
BLOCK_TORSION="${BLOCK_TORSION:-0.5}"
BLOCK_SLIP1="${BLOCK_SLIP1:-0.0}"
BLOCK_SLIP2="${BLOCK_SLIP2:-0.0}"
BLOCK_KP="${BLOCK_KP:-1e5}"
BLOCK_KD="${BLOCK_KD:-10.0}"
BLOCK_MIN_DEPTH="${BLOCK_MIN_DEPTH:-1e-5}"
BLOCK_MAX_VEL="${BLOCK_MAX_VEL:-0.10}"

ALLOW_RENAME="${ALLOW_RENAME:-false}"
SWEEP_SUFFIXES="${SWEEP_SUFFIXES:-true}"
MAX_SUFFIX="${MAX_SUFFIX:-9}"
WORLD_SDF="${WORLD_SDF:-/tmp/mastermind_empty_usercmd.world.sdf}"


# SCRIPT_PATH=$(readlink -f "$0")
# SCRIPT_DIR=$(dirname "$SCRIPT_PATH")
# MODELS_DIR="$SCRIPT_DIR/../models"
# export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$MODELS_DIR"
# echo "[DIAGNOSTIC] Exported GZ_SIM_RESOURCE_PATH: $MODELS_DIR"

# echo "[DIAGNOSTIC] Exported GZ_SIM_RESOURCE_PATH: ${PWD}/../models"
# export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:${PWD}/../models"
# -------------------------------------------------

need() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: $1 not found"; exit 1; }; }
need gz
need python3

ensure_world() {
  if ! gz service -l | grep -q "/world/"; then
    cat > "$WORLD_SDF" <<'SDF'
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="empty">
    <gravity>0 0 -9.81</gravity>
    <plugin name="scene_broadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="user_commands" filename="gz-sim-user-commands-system"/>
    <plugin name="sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
  </world>
</sdf>
SDF
    nohup gz sim -r "$WORLD_SDF" >/tmp/mastermind_world.log 2>&1 &
    sleep 0.5
  fi

  if ! gz service -l | grep -q "/world/${WORLD}/create"; then
    DETECTED=$(gz service -l | sed -n 's#.*/world/\([^/]*\)/create.*#\1#p' | head -n1 || true)
    [[ -n "$DETECTED" ]] && WORLD="$DETECTED"
  fi
  echo "[spawn] Using world: ${WORLD}"

  echo -n "[spawn] Waiting for /world/${WORLD}/set_pose ... "
  for _ in {1..60}; do
    if gz service -l | grep -q "/world/${WORLD}/set_pose"; then
      echo "OK"
      return
    fi
    sleep 0.25
  done
  echo "TIMEOUT"
  exit 1
}

USE_LONG_FLAGS=false
if gz service --help 2>/dev/null | grep -q -- '--reqtype'; then
  USE_LONG_FLAGS=true
fi

call_create() {
  local sdf_path="$1" name="$2" x="$3" y="$4" z="$5"
  local payload
  payload=$(cat <<PBUF
sdf_filename: "$sdf_path"
name: "$name"
pose { position { x: $x y: $y z: $z } }
allow_renaming: ${ALLOW_RENAME}
PBUF
)
  if $USE_LONG_FLAGS; then
    gz service -s "/world/${WORLD}/create" \
      --reqtype gz.msgs.EntityFactory \
      --reptype gz.msgs.Boolean \
      --timeout 3000 \
      --req "$payload" >/dev/null
  else
    gz service -s "/world/${WORLD}/create" \
      -m gz.msgs.EntityFactory \
      -r gz.msgs.Boolean \
      -p "$payload" >/dev/null
  fi
}

remove_model_once() {
  local name="$1"
  local payload=$'name: "'"$name"$'"\n'"type: MODEL"
  if $USE_LONG_FLAGS; then
    gz service -s "/world/${WORLD}/remove" \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
      --req "$payload" >/dev/null 2>&1 || true
  else
    gz service -s "/world/${WORLD}/remove" \
      -m gz.msgs.Entity -r gz.msgs.Boolean -p "$payload" >/dev/null 2>&1 || true
  fi
}

remove_model() {
  local base="$1"
  remove_model_once "$base"
  if [[ "$SWEEP_SUFFIXES" == "true" ]]; then
    for i in $(seq 1 "$MAX_SUFFIX"); do
      remove_model_once "${base}_${i}"
    done
  fi
}

SDF_DIR=/tmp/mastermind_spawn
mkdir -p "$SDF_DIR"

export BLOCK_SIZE BLOCK_MASS BLOCK_DENSITY
BLOCK_MASS_COMPUTED=$(python3 - <<'PY'
import os
a=float(os.environ.get("BLOCK_SIZE","0.04"))
dens=os.environ.get("BLOCK_DENSITY","").strip()
if dens:
    m=float(dens)*(a**3)
else:
    m=float(os.environ.get("BLOCK_MASS","0.02"))
print(f"{m:.8f}")
PY
)
export BLOCK_MASS_COMPUTED
BLOCK_INERTIA=$(python3 - <<'PY'
import os
a=float(os.environ.get("BLOCK_SIZE","0.04"))
m=float(os.environ.get("BLOCK_MASS_COMPUTED","0.02"))
I=(m*(a**2))/6.0
print(f"{I:.8e}")
PY
)
export BLOCK_INERTIA

TABLE_SDF="${SDF_DIR}/table_${TABLE_SIZE_X}x${TABLE_SIZE_Y}x${TABLE_THICK}.sdf"
cat > "$TABLE_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="mastermind_table">
    <static>true</static>
    <link name="top">
      <collision name="c">
        <geometry><box><size>${TABLE_SIZE_X} ${TABLE_SIZE_Y} ${TABLE_THICK}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>${TABLE_SIZE_X} ${TABLE_SIZE_Y} ${TABLE_THICK}</size></box></geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF


#camera stick, black
STICK_SDF="${SDF_DIR}/mastermind_camera_stick.sdf"
cat > "$STICK_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="mastermind_camera_stick">
    <static>true</static>
    <link name="stick">
      <pose>0.6 0 0 0 0 0</pose>
      
      <visual name="visual">
        <pose>0 0 0.3 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry>
        <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
      </visual>
      
      <collision name="col">
        <pose>0 0 0.3 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF


# camera, orange
CAMERA_SDF="${SDF_DIR}/camera_box.sdf"
cat > "$CAMERA_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="mastermind_camera_box">
    <static>true</static>
    
    <link name="camera_link">
      <pose>0.6 0 0.6 0 0.9 3.1415</pose>
      
      <visual name="visual">
        <geometry><box><size>0.025 0.09 0.025</size></box></geometry>
        <material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material>
      </visual>

      <sensor name="mastermind_camera" type="camera">
        <pose>0.02 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>10</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>mastermind/camera/image_raw</topic>
      </sensor>
    </link>
  </model>
</sdf>
EOF

write_cube_sdf() {
  local path="$1" r="$2" g="$3" b="$4"
  cat > "$path" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cube_40mm">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>${BLOCK_MASS_COMPUTED}</mass>
        <inertia>
          <ixx>${BLOCK_INERTIA}</ixx>
          <iyy>${BLOCK_INERTIA}</iyy>
          <izz>${BLOCK_INERTIA}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><box><size>${BLOCK_SIZE} ${BLOCK_SIZE} ${BLOCK_SIZE}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>${BLOCK_SIZE} ${BLOCK_SIZE} ${BLOCK_SIZE}</size></box></geometry>
        <material>
          <ambient>${r} ${g} ${b} 1</ambient>
          <diffuse>${r} ${g} ${b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
}

CUBE_RED_SDF="${SDF_DIR}/cube_red.sdf";     write_cube_sdf "$CUBE_RED_SDF"     0.85 0.10 0.10
CUBE_BLUE_SDF="${SDF_DIR}/cube_blue.sdf";   write_cube_sdf "$CUBE_BLUE_SDF"    0.10 0.10 0.85
CUBE_YELL_SDF="${SDF_DIR}/cube_yellow.sdf"; write_cube_sdf "$CUBE_YELL_SDF"    0.90 0.85 0.10
CUBE_GREEN_SDF="${SDF_DIR}/cube_green.sdf"; write_cube_sdf "$CUBE_GREEN_SDF"    0.1 0.85 0.1
CUBE_PURP_SDF="${SDF_DIR}/cube_purple.sdf"; write_cube_sdf "$CUBE_PURP_SDF"    0.50 0.00 0.50
CUBE_BLK_SDF="${SDF_DIR}/cube_black.sdf";   write_cube_sdf "$CUBE_BLK_SDF"     0.10 0.10 0.10

#gameboard
GAMEBOARD_SDF="${SDF_DIR}/gameboard.sdf"
cat > "$GAMEBOARD_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="mastermind_gameboard">
    <static>true</static>
    
    <link name="plate">
      <collision name="c">
        <geometry>
          <box><size>0.30 0.4 0.01</size></box>
        </geometry>
      </collision>
      <visual name="v">
        <geometry>
          <box><size>0.30 0.4 0.01</size></box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>

    # <link name="well_1">
    #   <pose>0.0 -0.075 0.0076 0 0 0</pose>
    #   <visual name="v">
    #     <geometry><cylinder><radius>0.015</radius><length>0.005</length></cylinder></geometry>
    #     <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
    #   </visual>
    # </link>

    # <link name="well_2">
    #   <pose>0.0 -0.025 0.0076 0 0 0</pose>
    #   <visual name="v">
    #     <geometry><cylinder><radius>0.015</radius><length>0.005</length></cylinder></geometry>
    #     <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
    #   </visual>
    # </link>

    # <link name="well_3">
    #   <pose>0.0 0.025 0.0076 0 0 0</pose>
    #   <visual name="v">
    #     <geometry><cylinder><radius>0.015</radius><length>0.005</length></cylinder></geometry>
    #     <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
    #   </visual>
    # </link>

    # <link name="well_4">
    #   <pose>0.0 0.075 0.0076 0 0 0</pose>
    #   <visual name="v">
    #     <geometry><cylinder><radius>0.015</radius><length>0.005</length></cylinder></geometry>
    #     <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
    #   </visual>
    # </link>

  </model>
</sdf>
EOF

ensure_world

echo "[spawn] Cleaning up previous mastermind entities…"
remove_model mastermind_table
remove_model mastermind_block_red
remove_model mastermind_block_blue
remove_model mastermind_block_yellow
remove_model mastermind_block_green
remove_model mastermind_block_purple
remove_model mastermind_block_black
remove_model mastermind_gameboard
remove_model mastermind_camera_box
remove_model mastermind_camera_stick

TABLE_CENTER_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}; th=${TABLE_THICK}
print(top - th/2.0)
PY
)
call_create "$TABLE_SDF" "mastermind_table" 0.0 0.0 "$TABLE_CENTER_Z"


echo "[spawn] Spawning Camera Stick (Black)..."
call_create "$STICK_SDF" "mastermind_camera_stick" 0.0 0.0 0.0

echo "[spawn] Spawning Camera Box (Orange)..."
call_create "$CAMERA_SDF" "mastermind_camera_box" 0.0 0.0 0.0


BLOCK_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}; sz=${BLOCK_SIZE}; drop=${BLOCK_DROP}
print(top + sz/2.0 + drop)
PY
)

# -------------------- 6 cubes -----------------------

FIXED_Y="-0.25"
START_X="-0.16"
SPACING="0.08"

echo "[spawn] Spawning Cubes on the Right Side..."

# 1. Red (X = -0.16)
POS_X_1=$(python3 -c "print($START_X)")
call_create "$CUBE_RED_SDF"    "mastermind_block_red"    "$POS_X_1"  "$FIXED_Y" "$BLOCK_Z"

# 2. Blue (X = -0.08)
POS_X_2=$(python3 -c "print($START_X + $SPACING)")
call_create "$CUBE_BLUE_SDF"   "mastermind_block_blue"   "$POS_X_2"  "$FIXED_Y" "$BLOCK_Z"

# 3. Yellow (X = 0.00)
POS_X_3=$(python3 -c "print($START_X + 2*$SPACING)")
call_create "$CUBE_YELL_SDF"   "mastermind_block_yellow" "$POS_X_3"  "$FIXED_Y" "$BLOCK_Z"

# 4. Green (X = 0.08)
POS_X_4=$(python3 -c "print($START_X + 3*$SPACING)")
call_create "$CUBE_GREEN_SDF"  "mastermind_block_green"  "$POS_X_4"  "$FIXED_Y" "$BLOCK_Z"

# 5. Purple (X = 0.16)
POS_X_5=$(python3 -c "print($START_X + 4*$SPACING)")
call_create "$CUBE_PURP_SDF"   "mastermind_block_purple" "$POS_X_5"  "$FIXED_Y" "$BLOCK_Z"

# 6. Black (X = 0.24)
POS_X_6=$(python3 -c "print($START_X + 5*$SPACING)")
call_create "$CUBE_BLK_SDF"    "mastermind_block_black"  "$POS_X_6"  "$FIXED_Y" "$BLOCK_Z"


# ----------------- 24 cubes-----------------

# echo "[spawn] Spawning 24 Cubes (6 Colors x 4 Blocks)..."


# # X-Axis: Controls the COLOR groups
# COLOR_START_X="-0.20"
# COLOR_SPACING_X="0.06"

# # Y-Axis: Controls the 4 BLOCKS of the same color
# BLOCK_START_Y="-0.25"
# BLOCK_SPACING_Y="-0.070"

# # Define a function to spawn a column (4) of same-colored blocks
# # Args: 1=Color Name, 2=SDF File Variable, 3=Color Index (0-5)
# spawn_row() {
#     local color_name="$1"
#     local sdf_file="$2"
#     local color_idx="$3"

#     # Calculate X coordinate for this COLOR group
#     # X = StartX + (ColorIndex * Spacing)
#     local x_pos=$(python3 -c "print($COLOR_START_X + $color_idx * $COLOR_SPACING_X)")

#     # Loop to spawn 4 blocks along the Y axis
#     for i in {1..4}; do
#         # Calculate Y coordinate for current block
#         # Y = StartY + (i-1) * Spacing
#         local y_pos=$(python3 -c "print($BLOCK_START_Y + ($i - 1) * $BLOCK_SPACING_Y)")
        
#         # Naming convention: mastermind_block_red_1, mastermind_block_red_2 ...
#         local name="mastermind_block_${color_name}_${i}"
        
#         call_create "$sdf_file" "$name" "$x_pos" "$y_pos" "$BLOCK_Z"
#     done
# }

# # --- Start spawning each color ---
# # Now the "Row" index controls the X position (Color Strip)

# # Strip 1: Red
# spawn_row "red"    "$CUBE_RED_SDF"   0

# # Strip 2: Blue
# spawn_row "blue"   "$CUBE_BLUE_SDF"  1

# # Strip 3: Yellow
# spawn_row "yellow" "$CUBE_YELL_SDF"  2

# # Strip 4: Green
# spawn_row "green"  "$CUBE_GREEN_SDF" 3

# # Strip 5: Purple
# spawn_row "purple" "$CUBE_PURP_SDF"  4

# # Strip 6: Black 
# spawn_row "black"  "$CUBE_BLK_SDF"   5

#----------------------------------------------------------



# spawn the gameboard so it sits ON the table
# Position: X=0.30 (Front of robot), Y=0.0 (Centered)
PLATE_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}
th=0.02
print(top + th/2.0)
PY
)
call_create "$GAMEBOARD_SDF" "mastermind_gameboard" 0.30 0.0 "$PLATE_Z"



#--------vision test boxes on gameboard----------

echo "[spawn] Placing 4 target blocks into the gameboard wells..."


BLOCK_ON_BOARD_Z=$(python3 -c "print($BLOCK_Z + 0.01)")

# Gameboard Center is X=0.30
BOARD_X="0.30"

# Pos 1: Y = -0.075 (Red)
call_create "$CUBE_RED_SDF"  "target_block_1" "$BOARD_X" "-0.12" "$BLOCK_ON_BOARD_Z"

# Pos 2: Y = -0.025 (Blue)
call_create "$CUBE_BLUE_SDF" "target_block_2" "$BOARD_X" "-0.04" "$BLOCK_ON_BOARD_Z"

# Pos 3: Y = 0.025 (Green)
call_create "$CUBE_GREEN_SDF" "target_block_3" "$BOARD_X" "0.04"  "$BLOCK_ON_BOARD_Z"

# Pos 4: Y = 0.075 (Yellow)
call_create "$CUBE_YELL_SDF" "target_block_4" "$BOARD_X" "0.12"  "$BLOCK_ON_BOARD_Z"

#----------------------------


echo "[spawn] Done."