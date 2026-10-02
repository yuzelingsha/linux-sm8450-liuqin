#!/usr/bin/env bash
set -euo pipefail
source_dir="$GITHUB_WORKSPACE/linux"
out="$GITHUB_WORKSPACE/build"
mkdir -p "$out"
cp "$GITHUB_WORKSPACE/ci-harness/ci/fpc1264.config" "$out/.config"
expected_config=$(sha256sum "$out/.config" | cut -d' ' -f1)
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
export KBUILD_BUILD_USER=liuqin KBUILD_BUILD_HOST=github-actions KBUILD_BUILD_VERSION=1
export KBUILD_BUILD_TIMESTAMP=$(git -C "$source_dir" show -s --format=%cI HEAD)
make_args=(-C "$source_dir" O="$out" RUSTC=false)
make "${make_args[@]}" olddefconfig
actual_config=$(sha256sum "$out/.config" | cut -d' ' -f1)
[[ "$actual_config" == "$expected_config" ]]
for setting in CONFIG_TEE=y CONFIG_TEE_QSEECOM=y CONFIG_QCOM_QSEECOM=y CONFIG_QCOM_MDT_LOADER=y CONFIG_SENSORS_FPC1020_SPI=m; do
    grep -Fx "$setting" "$out/.config"
done
grep -Fx '# CONFIG_KASAN is not set' "$out/.config"
make "${make_args[@]}" -j4 Image qcom/sm8475-xiaomi-liuqin.dtb modules
[[ -s "$out/drivers/misc/fpc1264_spi_diag.ko" ]]
[[ -s "$out/drivers/tee/qseecom/core.o" ]]
make "${make_args[@]}" -j4 KCFLAGS=-Werror drivers/misc/fpc1264_spi_diag.o
make "${make_args[@]}" modules_install INSTALL_MOD_PATH="$out/module-install" INSTALL_MOD_STRIP=1
find "$out/module-install" -type l -delete
sha=$(git -C "$source_dir" rev-parse HEAD)
[[ "$sha" == "$TESTED_REVISION" ]]
[[ -z $(git -C "$source_dir" status --porcelain) ]]
export TESTED_CONFIG_SHA256="$actual_config"
python3 - <<'PY'
import json,os,pathlib,subprocess,hashlib
out=pathlib.Path(os.environ['GITHUB_WORKSPACE'])/'build'
paths=['arch/arm64/boot/Image','arch/arm64/boot/dts/qcom/sm8475-xiaomi-liuqin.dtb','drivers/misc/fpc1264_spi_diag.ko','.config','System.map','vmlinux']
hashes={p:hashlib.sha256((out/p).read_bytes()).hexdigest() for p in paths}
data={'kernel_commit':os.environ['TESTED_REVISION'],'config_sha256':os.environ['TESTED_CONFIG_SHA256'],'compiler':subprocess.check_output(['aarch64-linux-gnu-gcc','--version'],text=True).splitlines()[0],'feature_build':{'TEE_QSEECOM':'y','SENSORS_FPC1020_SPI':'m'},'module_count':len(list((out/'module-install').rglob('*.ko'))),'sha256':hashes,'device_acceptance':False}
(out/'CI_BUILD.json').write_text(json.dumps(data,indent=2)+'\n')
(out/'SHA256SUMS').write_text(''.join(v+'  '+k+'\n' for k,v in hashes.items()))
PY
