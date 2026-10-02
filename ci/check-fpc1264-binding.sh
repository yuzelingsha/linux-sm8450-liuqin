#!/usr/bin/env bash
set -euo pipefail
source_dir="$GITHUB_WORKSPACE/linux"
out="$GITHUB_WORKSPACE/schema-out"
mkdir -p "$out"
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
export PATH="$RUNNER_TEMP/dt-schema/bin:$PATH"
make_args=(-C "$source_dir" O="$out" RUSTC=false)
make "${make_args[@]}" defconfig
make "${make_args[@]}" -j4 dt_binding_check DT_SCHEMA_FILES=input/fpc,fpc1020.yaml 2>&1 | tee "$out/binding-check.log"
make "${make_args[@]}" -j4 CHECK_DTBS=y DT_SCHEMA_FILES=input/fpc,fpc1020.yaml qcom/sm8475-xiaomi-liuqin.dtb 2>&1 | tee "$out/dtb-check.log"
printf 'kernel_commit=%s\ndtschema_version=%s\n' "$TESTED_REVISION" "$(python3 -c 'import importlib.metadata; print(importlib.metadata.version("dtschema"))')" > "$out/CI_SCHEMA.txt"
