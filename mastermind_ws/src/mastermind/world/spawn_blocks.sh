#!/usr/bin/env bash
set -euo pipefail

# ---------- Tunables (override via env) ----------
WORLD="${WORLD:-empty}"
TABLE_TOP_Z="${TABLE_TOP_Z:-0.30}"
TABLE_SIZE_X="${TABLE_SIZE_X:-1.0}"
TABLE_SIZE_Y="${TABLE_SIZE_Y:-1.0}"
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
MAX_SUFFIX="${MAX_SUFFIX:-1}"
WORLD_SDF="${WORLD_SDF:-/tmp/lab06_empty_usercmd.world.sdf}"
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
  </world>
</sdf>
SDF
    nohup gz sim -r "$WORLD_SDF" >/tmp/lab06_world.log 2>&1 &
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


SDF_DIR=/tmp/lab06_spawn
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
  <model name="lab06_table">
    <static>true</static>
    <link name="top">
      <collision name="c">
        <geometry><box><size>${TABLE_SIZE_X} ${TABLE_SIZE_Y} ${TABLE_THICK}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>${TABLE_SIZE_X} ${TABLE_SIZE_Y} ${TABLE_THICK}</size></box></geometry>
      </visual>
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
CUBE_GREEN_SDF="${SDF_DIR}/cube_green.sdf"; write_cube_sdf "$CUBE_GREEN_SDF"   0.10 0.85 0.10
CUBE_PURP_SDF="${SDF_DIR}/cube_purple.sdf"; write_cube_sdf "$CUBE_PURP_SDF"    0.60 0.10 0.60
CUBE_BLACK_SDF="${SDF_DIR}/cube_black.sdf"; write_cube_sdf "$CUBE_BLACK_SDF"   0.02 0.02 0.02


# white board sdf (thin plate) at x=0.20, y=0.32
WHITE_SDF="${SDF_DIR}/white_plate.sdf"
cat > "$WHITE_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="lab06_white_plate">
    <static>true</static>
    <link name="plate">
      <collision name="c">
        <geometry>
          <box><size>0.25 0.30 0.01</size></box>
        </geometry>
      </collision>
      <visual name="v">
        <geometry>
          <box><size>0.25 0.30 0.01</size></box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF


# # ---------- Overhead camera SDF (lab06_overhead_camera) ----------
# CAMERA_SDF="${SDF_DIR}/lab06_overhead_camera.sdf"
# cat > "$CAMERA_SDF" <<EOF
# <?xml version="1.0" ?>
# <sdf version="1.8">
#   <model name="lab06_overhead_camera">
#     <static>true</static>
#     <link name="camera_link">
#       <!-- Pose: x y z roll pitch yaw -->
#       <!-- Centered above the white plate at (0.16, 0.32), looking down -->
#       <pose>0.16 0.32 0.80 0 1.57 0</pose>

#       <visual name="body">
#         <geometry>
#           <box><size>0.05 0.05 0.05</size></box>
#         </geometry>
#         <material>
#           <ambient>1 0.5 0 1</ambient>
#           <diffuse>1 0.5 0 1</diffuse>
#         </material>
#       </visual>

#       <sensor name="overhead_camera" type="camera">
#         <pose>0 0 0 0 0 0</pose>
#         <camera>
#           <horizontal_fov>1.047</horizontal_fov>
#           <image>
#             <width>640</width>
#             <height>480</height>
#             <format>R8G8B8</format>
#           </image>
#           <clip>
#             <near>0.1</near>
#             <far>10.0</far>
#           </clip>
#         </camera>
#         <always_on>1</always_on>
#         <update_rate>15</update_rate>
#         <visualize>true</visualize>
#       </sensor>
#     </link>
#   </model>
# </sdf>
# EOF
# # ----------------------------------------------------

#camera stick, black
STICK_SDF="${SDF_DIR}/mastermind_camera_stick.sdf"
cat > "$STICK_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="mastermind_camera_stick">
    <static>true</static>
    <link name="stick">
      <pose>0.6 0.32 0 0 0 0</pose>
      
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
      <pose>0.6 0.32 0.6 0.0 0.7 3.1415</pose>
      
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
        <update_rate>1</update_rate>
        <visualize>true</visualize>
        <topic>mastermind/camera/image_raw</topic>
      </sensor>
    </link>
  </model>
</sdf>
EOF


ensure_world


# echo "[spawn] Cleaning up previous lab06 entities…"
# remove_model lab06_table
# remove_model lab06_block_red
# remove_model lab06_block_blue
# remove_model lab06_block_yellow
# remove_model lab06_block_green
# remove_model lab06_block_purple
# remove_model lab06_block_black
# remove_model lab06_white_plate
# remove_model lab06_overhead_camera


TABLE_CENTER_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}; th=${TABLE_THICK}
print(top - th/2.0)
PY
)
call_create "$TABLE_SDF" "lab06_table" 0.0 0.0 "$TABLE_CENTER_Z"


BLOCK_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}; sz=${BLOCK_SIZE}; drop=${BLOCK_DROP}
print(top + sz/2.0 + drop)
PY
)

# first 4 blocks: EXACTLY your original layout (do not change)

call_create "$CUBE_BLUE_SDF"  "lab06_block_blue"   "0.45" "0.0"        "$BLOCK_Z"  # y =  0.00
call_create "$CUBE_YELL_SDF"  "lab06_block_yellow" "0.45" "-0.08"       "$BLOCK_Z"  # y =  0.08
call_create "$CUBE_GREEN_SDF" "lab06_block_green"  "0.45" "-0.16"       "$BLOCK_Z"  # y =  0.16

# purple & black, unchanged from your script
call_create "$CUBE_RED_SDF"   "lab06_block_red"    "0.33" "-0.08"       "$BLOCK_Z"  # y = -0.08
call_create "$CUBE_PURP_SDF"  "lab06_block_purple" "0.33" "0.0"       "$BLOCK_Z"
call_create "$CUBE_BLACK_SDF" "lab06_block_black"  "0.33" "-0.16"        "$BLOCK_Z"


# spawn the white board so it sits ON the table (top at 0.30)
PLATE_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}
th=0.01
print(top - th/2.0 + 0.01)
PY
)
call_create "$WHITE_SDF" "lab06_white_plate" 0.16 0.32 "$PLATE_Z"


# # spawn the overhead camera (pose is defined inside SDF)
# echo "[spawn] Spawning overhead camera..."
# call_create "$CAMERA_SDF" "lab06_overhead_camera" 0.0 0.0 0.0

echo "[spawn] Spawning Camera Stick (Black)..."
call_create "$STICK_SDF" "mastermind_camera_stick" 0.0 0.0 0.0

echo "[spawn] Spawning Camera Box (Orange)..."
call_create "$CAMERA_SDF" "mastermind_camera_box" 0.0 0.0 0.0


echo "[spawn] Done."
