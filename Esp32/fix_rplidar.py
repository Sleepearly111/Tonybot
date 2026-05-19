"""
PlatformIO extra_script: 修复 RPLIDAR 库在 ESP32 上的编译问题。

RoboPeak 库的 begin() 声明为 bool 但未返回有效值，
ESP32 编译器会报 warning/warning-as-error。这里把
bool → void 修正。
"""

import os


def patch_file(path, old, new):
    if not os.path.isfile(path):
        return False
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()
    if old not in content:
        return False  # 可能已修复或版本不同
    content = content.replace(old, new)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    return True


def apply_patches(env):
    # 确定库的实际路径
    project_dir = env.subst("$PROJECT_DIR")
    lib_dir = os.path.join(project_dir, ".pio", "libdeps", env.subst("$PIOENV"))

    # 递归搜索 RPLidar.h 和 RPLidar.cpp
    for root, _dirs, files in os.walk(lib_dir):
        for fname in files:
            fpath = os.path.join(root, fname)

            if fname == "RPLidar.h":
                ok = patch_file(
                    fpath,
                    "bool begin(HardwareSerial &serialobj);",
                    "void begin(HardwareSerial &serialobj);",
                )
                if ok:
                    print(f"[fix_rplidar] 已修复: {fpath}")

            elif fname == "RPLidar.cpp":
                ok = patch_file(
                    fpath,
                    "bool RPLidar::begin(HardwareSerial &serialobj)",
                    "void RPLidar::begin(HardwareSerial &serialobj)",
                )
                if ok:
                    print(f"[fix_rplidar] 已修复: {fpath}")


Import("env")
apply_patches(env)
