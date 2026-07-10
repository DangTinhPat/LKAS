Import("env")

# Workaround for a micro_ros_platformio + newer Arduino-ESP32 core (pioarduino,
# which bundles ESP-Matter/connectedhomeip) incompatibility: the framework injects
# a CCFLAGS entry like -DCHIP_ADDRESS_RESOLVE_IMPL_INCLUDE_HEADER=<some/header.h>
# (angle brackets, only relevant to the ESP-Matter component). CMake's Unix
# Makefiles generator mishandles the '<'/'>' in that token when it flows into
# CMAKE_C_FLAGS_INIT: it re-emits it into flags.make wrapped in stray literal
# ";" characters, which breaks the shell CMake invokes to run the compiler
# ("Syntax error: ';' unexpected"). The flag is meaningless to micro-ROS's own
# build (rcutils/microcdr/etc. never touch ESP-Matter headers), so instead of
# trying to re-escape it, just drop any flag containing '<' before it reaches
# the toolchain file.
#
# Patch the vendored extra_script.py in-place (before it's imported by the
# library-dependency finder). Idempotent — matches either the pristine upstream
# text or an earlier (semicolon-replace only) version of this same patch.
import os

_path = os.path.join(
    env["PROJECT_LIBDEPS_DIR"], env["PIOENV"], "micro_ros_platformio", "extra_script.py"
)

if os.path.isfile(_path):
    with open(_path, "r") as f:
        content = f.read()

    _new_cflags = "\"{} {} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='\".format(' '.join(f for f in env['CFLAGS'] if '<' not in f), ' '.join(f for f in env['CCFLAGS'] if '<' not in f)),"
    _new_cxxflags = "\"{} {} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='\".format(' '.join(f for f in env['CXXFLAGS'] if '<' not in f), ' '.join(f for f in env['CCFLAGS'] if '<' not in f))"

    _candidates_cflags = [
        "\"{} {} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='\".format(' '.join(env['CFLAGS']), ' '.join(env['CCFLAGS'])),",
        "\"{} {} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='\".format(' '.join(f.replace(';', ' ') for f in env['CFLAGS']), ' '.join(f.replace(';', ' ') for f in env['CCFLAGS'])),",
    ]
    _candidates_cxxflags = [
        "\"{} {} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='\".format(' '.join(env['CXXFLAGS']), ' '.join(env['CCFLAGS']))",
        "\"{} {} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='\".format(' '.join(f.replace(';', ' ') for f in env['CXXFLAGS']), ' '.join(f.replace(';', ' ') for f in env['CCFLAGS']))",
    ]

    changed = False
    for old in _candidates_cflags:
        if old in content:
            content = content.replace(old, _new_cflags)
            changed = True
            break
    for old in _candidates_cxxflags:
        if old in content:
            content = content.replace(old, _new_cxxflags)
            changed = True
            break

    if changed:
        with open(_path, "w") as f:
            f.write(content)
        print("[fix_matter_ccflags] patched micro_ros_platformio/extra_script.py to drop '<'-bearing CCFLAGS")
