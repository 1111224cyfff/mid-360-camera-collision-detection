# YOLO Training Environment Snapshot (2026-04-15)

## Device
- Model: NVIDIA Orin NX Developer Kit
- Architecture: aarch64
- CPU: 8 cores, ARMv8 Processor rev 1 (v8l)
- CPU Frequency: 115.2 MHz to 1984.0 MHz
- RAM: 15 GiB total
- Swap: 7.6 GiB total
- Root Disk: 116G total, 89G used, 21G available (81% used)

## Operating System
- Ubuntu: 20.04.6 LTS (Focal Fossa)
- Kernel: Linux 5.10.120-tegra

## Jetson / NVIDIA Stack
- L4T Release: R35.4.1
- /etc/nv_tegra_release: `# R35 (release), REVISION: 4.1, GCID: 33958178, BOARD: t186ref, EABI: aarch64, DATE: Tue Aug 1 19:57:35 UTC 2023`
- nvidia-smi: not available on this Jetson setup (normal on many Jetson environments)

## CUDA / cuDNN / TensorRT
- CUDA Compiler (nvcc): 11.4.315
- CUDA Toolkit package: cuda-toolkit-11-4 11.4.19-1
- cuDNN: libcudnn8 8.6.0.166-1+cuda11.4
- TensorRT runtime: libnvinfer8 8.5.2-1+cuda11.4
- TensorRT dev: libnvinfer-dev 8.5.2-1+cuda11.4
- TensorRT Python package: tensorrt 8.5.2.2

## Python Environment
- Type: VirtualEnvironment
- Python: 3.8.10
- Environment path: /home/nvidia/ws_livox/.venv
- Pip: 25.0.1

## YOLO and Core ML Packages
- ultralytics: 8.4.21
- torch: 2.2.0
- torchvision: 0.17.2+c1d70fe
- onnx: 1.17.0
- onnxruntime: 1.16.3
- onnxruntime-gpu: 1.17.0
- opencv-python: 4.13.0.92
- tensorflow: 2.13.1

## Build Toolchain
- GCC: 9.4.0
- CMake: 3.16.3

## Notes
- `yolo --version` returned 8.4.21.
- In this workspace, recent commands indicate successful model export and TensorRT metadata wrapping.
