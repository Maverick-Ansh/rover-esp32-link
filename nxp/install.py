"""Copy the link-test firmware into an MCUXpresso SDK tree as the example `demo_apps/esp_link`.

    python nxp/install.py --sdk <your west workspace>/mcuxsdk
    cd <your west workspace>
    west build -b frdmmcxn947 mcuxsdk/examples/demo_apps/esp_link -d build_out -p always -- -Dcore_id=cm33_core0

The SDK's build system only finds examples inside its own tree, so the sources live here in the
repo and this script places them. It overwrites these two folders and nothing else:
  examples/demo_apps/esp_link/                       main.c, CMakeLists.txt, example.yml, prj.conf,
                                                     rover_link.h (from esp32cam/LinkTest/)
  examples/_boards/frdmmcxn947/demo_apps/esp_link/   pin_mux.c/.h, reconfig.cmake,
                                                     cm33_core0/{app.h, hardware_init.c, prj.conf}
Re-run it after every edit to the firmware or to rover_link.h.
"""
import argparse, os, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
PROTOCOL = os.path.join(HERE, "..", "esp32cam", "LinkTest", "rover_link.h")

ap = argparse.ArgumentParser()
ap.add_argument("--sdk", default="D:/mcuxsdk/mcuxsdk",
                help="the mcuxsdk folder INSIDE the west workspace (the one holding examples/)")
args = ap.parse_args()

if not os.path.isdir(os.path.join(args.sdk, "examples", "_boards", "frdmmcxn947")):
    raise SystemExit(f"{args.sdk} does not look like an MCUXpresso SDK with the frdmmcxn947 board")

app = os.path.join(args.sdk, "examples", "demo_apps", "esp_link")
board = os.path.join(args.sdk, "examples", "_boards", "frdmmcxn947", "demo_apps", "esp_link")
for d in (app, board):
    shutil.rmtree(d, ignore_errors=True)
shutil.copytree(os.path.join(HERE, "app"), app)
shutil.copy2(PROTOCOL, app)
shutil.copytree(os.path.join(HERE, "board"), board)
print(f"installed\n  {app}\n  {board}")
