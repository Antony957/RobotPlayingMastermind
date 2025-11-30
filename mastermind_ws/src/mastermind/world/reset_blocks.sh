#!/usr/bin/env bash
set -euo pipefail

WORLD="${WORLD:-empty}"
TABLE_TOP_Z="${TABLE_TOP_Z:-0.30}"
BLOCK_SIZE="${BLOCK_SIZE:-0.04}"
BLOCK_DROP="${BLOCK_DROP:-0.01}"

ALLOW_RENAME="${ALLOW_RENAME:-false}"
SWEEP_SUFFIXES="${SWEEP_SUFFIXES:-true}"
MAX_SUFFIX="${MAX_SUFFIX:-3}"

command -v gz >/dev/null 2>&1 || { echo "ERROR: gz (Ignition/Gazebo) not found"; exit 1; }

USE_LONG_FLAGS=false
if gz service --help 2>/dev/null | grep -q -- '--reqtype'; then
  USE_LONG_FLAGS=true
fi

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

write_cube_sdf() {
  local path="$1" r="$2" g="$3" b="$4"
  mkdir -p "$(dirname "$path")"
  cat > "$path" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cube_40mm">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.02</mass>
        <inertia>
          <ixx>1e-6</ixx>
          <iyy>1e-6</iyy>
          <izz>1e-6</izz>
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

# compute block Z
BLOCK_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}; sz=${BLOCK_SIZE}; drop=${BLOCK_DROP}
print(top + sz/2.0 + drop)
PY
)

# prepare SDFs
SDF_DIR=/tmp/lab06_spawn
mkdir -p "$SDF_DIR"
CUBE_RED_SDF="${SDF_DIR}/cube_red.sdf";     write_cube_sdf "$CUBE_RED_SDF"     0.85 0.10 0.10
CUBE_BLUE_SDF="${SDF_DIR}/cube_blue.sdf";   write_cube_sdf "$CUBE_BLUE_SDF"    0.10 0.10 0.85
CUBE_YELL_SDF="${SDF_DIR}/cube_yellow.sdf"; write_cube_sdf "$CUBE_YELL_SDF"    0.90 0.85 0.10
CUBE_GREEN_SDF="${SDF_DIR}/cube_green.sdf"; write_cube_sdf "$CUBE_GREEN_SDF"   0.10 0.85 0.10
CUBE_PURP_SDF="${SDF_DIR}/cube_purple.sdf"; write_cube_sdf "$CUBE_PURP_SDF"    0.60 0.10 0.60
CUBE_BLACK_SDF="${SDF_DIR}/cube_black.sdf"; write_cube_sdf "$CUBE_BLACK_SDF"   0.02 0.02 0.02

# ensure the world create/remove services are there
for _ in {1..40}; do
  if gz service -l | grep -q "/world/${WORLD}/create"; then break; fi
  sleep 0.05
done

echo "[reset] Deleting previous blocks..."
remove_model lab06_block_blue
remove_model lab06_block_yellow
remove_model lab06_block_green
remove_model lab06_block_red
remove_model lab06_block_purple
remove_model lab06_block_black

echo "[reset] Respawning canonical blocks..."
call_create "$CUBE_BLUE_SDF"  "lab06_block_blue"   "0.45" "0.0"  "$BLOCK_Z"
call_create "$CUBE_YELL_SDF"  "lab06_block_yellow" "0.45" "-0.08" "$BLOCK_Z"
call_create "$CUBE_GREEN_SDF" "lab06_block_green"  "0.45" "-0.16" "$BLOCK_Z"
call_create "$CUBE_RED_SDF"   "lab06_block_red"    "0.33" "-0.08" "$BLOCK_Z"
call_create "$CUBE_PURP_SDF"  "lab06_block_purple" "0.33" "0.0"  "$BLOCK_Z"
call_create "$CUBE_BLACK_SDF" "lab06_block_black"  "0.33" "-0.16" "$BLOCK_Z"

echo "[reset] Done."
