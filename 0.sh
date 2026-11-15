#!/bin/sh

SATURATION=145          # 100 = original, 130-160 is usually good for satellite
CONTRAST=4x45%          # sigmoidal-contrast strength (higher = more punch)
SHARPEN="0x0.6+0.6+0.02" # mild unsharp mask

IN_DIR="$1"
OUT_DIR="$2"
mkdir -p "$OUT_DIR"
for f in "$IN_DIR"/*.jpg; do
    base=`basename "$f"`
    echo $base
    magick "$f" \
        -colorspace sRGB \
        -auto-level \
        -modulate 100,"$SATURATION",100 \
        -sigmoidal-contrast "$CONTRAST" \
        -unsharp "$SHARPEN" \
        -quality 92 \
        "$OUT_DIR/$base"
done
