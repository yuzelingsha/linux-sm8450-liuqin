# Linux for Xiaomi Pad 6 Pro

Device support for the Xiaomi Pad 6 Pro (liuqin, Qualcomm SM8475), based on
[sm8450-mainline/linux](https://github.com/sm8450-mainline/linux).

The device branch is `liuqin-6.17`. Ubuntu integration, build configuration and
installation documentation are maintained in the companion
**xiaomipad-6pro-mainline** repository. Its `kernel/source.json` selects the exact
kernel revision used for each build.

Kernel changes are maintained here as Git commits. Use the companion project's
build guide to build matching kernel, device-tree and module outputs. A kernel
Image alone is not an Ubuntu installation image.

The AudioReach/TDM support includes work by Prasad Kumpatla and the Linux
Qualcomm community. Original authorship and file-level license notices are
retained. See `COPYING` and `LICENSES/` for the kernel's licensing terms.
