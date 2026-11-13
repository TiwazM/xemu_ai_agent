#!/bin/sh
# Refreshes this directory from the M65-MCP's repository: the reference model
# the hardware is verified against, and the part of Berkeley SoftFloat 3e the
# model is built on. Run it when the model there has changed.
#
#   targets/mega65/mcp65-model/sync.sh /path/to/m65-mcp
#
# Needs src/mcp/sim/fetch.sh run once over there (it fetches SoftFloat, pinned).
# Copies the files as they are, writes VERSIONS, and writes the list of
# SoftFloat's sources into ../mcp65_softfloat.c. platform.h is this
# directory's own and is not touched. If the model has come to use a SoftFloat
# function that is not in SF_SOURCES below, the link fails with its name: add
# its file (and what that needs) to the list.
set -e
here=$(cd "$(dirname "$0")" && pwd)
src=${1:?usage: sync.sh /path/to/m65-mcp}
model=$src/src/mcp/model
sf=$src/src/mcp/ext/berkeley-softfloat-3
spec=ARM-VFPv2-defaultNaN
[ -f "$model/mcp_model.c" ] || { echo "no $model/mcp_model.c"; exit 1; }
[ -d "$sf/source" ] || { echo "no $sf: run src/mcp/sim/fetch.sh there first"; exit 1; }

SF_SOURCES="softfloat_state
f32_add f32_sub f32_mul f32_mulAdd f32_div f32_sqrt f32_roundToInt
f32_eq f32_lt_quiet f32_to_i32 f32_to_f64 i32_to_f32
f64_mul f64_to_i32 f64_to_i64
s_addMagsF32 s_subMagsF32 s_mulAddF32
s_roundPackToF32 s_normRoundPackToF32 s_normSubnormalF32Sig
s_roundPackToF64 s_normSubnormalF64Sig
s_roundToI32 s_roundMToI64
s_shiftRightJamM s_shortShiftRightJamM s_mul64To128M
s_approxRecip32_1 s_approxRecip_1Ks s_approxRecipSqrt32_1 s_approxRecipSqrt_1Ks"
SF_SPECIALIZE="s_propagateNaNF32UI s_propagateNaNF64UI softfloat_raiseFlags"
SF_HEADERS="internals.h opts-GCC.h primitiveTypes.h primitives.h softfloat.h softfloat_types.h"

cp "$model/mcp_model.c" "$model/mcp_model.h" "$here/"
for f in $SF_SOURCES; do cp "$sf/source/$f.c" "$here/"; done
for f in $SF_SPECIALIZE; do cp "$sf/source/$spec/$f.c" "$here/"; done
for f in $SF_HEADERS; do cp "$sf/source/include/$f" "$here/"; done
cp "$sf/source/$spec/specialize.h" "$here/"
cp "$sf/COPYING.txt" "$here/SoftFloat-COPYING.txt"

{
    echo "mcp_model.c, mcp_model.h: $(git -C "$src" config --get remote.origin.url | sed 's/.*github.com[:\/]//; s/\.git$//') src/mcp/model"
    echo "    commit $(git -C "$src" log -1 --format='%H %cs' -- src/mcp/model/mcp_model.c src/mcp/model/mcp_model.h)"
    [ -z "$(git -C "$src" status --porcelain -- src/mcp/model/mcp_model.c src/mcp/model/mcp_model.h)" ] || echo "    (with uncommitted changes)"
    echo "SoftFloat: ucb-bar/berkeley-softfloat-3, specialisation $spec"
    echo "    commit $(git -C "$sf" log -1 --format='%H %cs')"
} > "$here/VERSIONS"

{
    echo "/* The part of Berkeley SoftFloat 3e that the M65-MCP's model needs, as one"
    echo " * compilation unit. Written by mcp65-model/sync.sh: do not edit. */"
    echo
    echo "#pragma GCC diagnostic ignored \"-Wunused-variable\""
    echo
    for f in $SF_SOURCES $SF_SPECIALIZE; do echo "#include \"mcp65-model/$f.c\""; done
} > "$here/../mcp65_softfloat.c"
cat "$here/VERSIONS"
