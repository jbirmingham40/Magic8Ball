"""
PlatformIO pre:link script — patch sections.ld and base sdkconfig after
HybridCompile rebuilds ESP-IDF libs.

HybridCompile regenerates sections.ld each time, dropping entries for
functions that only exist when CONFIG_PM_ENABLE / CONFIG_FREERTOS_USE_TICKLESS_IDLE
are enabled.  This script widens three narrow section listings to wildcards
so the linker places .literal before .text (required by Xtensa l32r).

It also ensures the base sdkconfig has CONFIG_SPIRAM_BOOT_HW_INIT=y so PSRAM
is initialised at boot (HybridCompile's kconfig merge doesn't reliably
propagate this from sdkconfig.defaults).
"""
import os, re

Import("env")

libs_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32-libs")
if not libs_dir:
    print("[patch_sections_ld] libs package not found — skipping")
else:
    mcu = env.BoardConfig().get("build.mcu", "esp32s3")
    mem_type = env.BoardConfig().get("build.arduino.memory_type",
                                     env.GetProjectOption("board_build.arduino.memory_type", "qio_opi"))

    sections_ld = os.path.join(libs_dir, mcu, mem_type, "sections.ld")
    base_sdkconfig = os.path.join(libs_dir, mcu, "sdkconfig")

    # ── 1.  Patch sections.ld ──────────────────────────────────────────
    if os.path.isfile(sections_ld):
        with open(sections_ld, "r") as f:
            text = f.read()

        changed = False

        # Fix 1: libesp_pm.a:pm_impl.* — replace specific function list with wildcard
        old_pm = re.search(
            r"(\*libesp_pm\.a:pm_impl\.\*\()\.literal\.esp_pm_get_configuration[^)]+\)",
            text)
        if old_pm:
            text = text[:old_pm.start()] + \
                   "*libesp_pm.a:pm_impl.*(.literal .literal.* .text .text.*)" + \
                   text[old_pm.end():]
            changed = True

        # Fix 2: libesp_hw_support.a:periph_ctrl.* — the NON-IRAM entry (has periph_ll_get_clk_en_mask)
        old_periph = re.search(
            r"(\*libesp_hw_support\.a:periph_ctrl\.\*\()\.literal\.periph_ll_get_clk_en_mask[^)]+\)",
            text)
        if old_periph:
            text = text[:old_periph.start()] + \
                   "*libesp_hw_support.a:periph_ctrl.*(.literal .literal.* .text .text.*)" + \
                   text[old_periph.end():]
            changed = True

        # Fix 3: add prvGetExpectedIdleTime + vTaskStepTick after last tasks.* entry
        if ".literal.prvGetExpectedIdleTime" not in text:
            # Find the last libfreertos.a:tasks.* line
            last_tasks = None
            for m in re.finditer(r"^\s+\*libfreertos\.a:tasks\.\*\(.*\)$", text, re.MULTILINE):
                last_tasks = m
            if last_tasks:
                insert = (
                    "\n    *libfreertos.a:tasks.*(.literal.prvGetExpectedIdleTime .text.prvGetExpectedIdleTime)"
                    "\n    *libfreertos.a:tasks.*(.literal.vTaskStepTick .text.vTaskStepTick)"
                )
                text = text[:last_tasks.end()] + insert + text[last_tasks.end():]
                changed = True

        if changed:
            with open(sections_ld, "w") as f:
                f.write(text)
            print("[patch_sections_ld] patched %s" % sections_ld)
        else:
            print("[patch_sections_ld] %s already patched" % sections_ld)

    # ── 2.  Patch base sdkconfig (SPIRAM boot init) ───────────────────
    if os.path.isfile(base_sdkconfig):
        with open(base_sdkconfig, "r") as f:
            cfg = f.read()

        patched = False
        for opt in [
            ("# CONFIG_SPIRAM_BOOT_HW_INIT is not set", "CONFIG_SPIRAM_BOOT_HW_INIT=y"),
            ("# CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION is not set",
             "CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=y"),
            # BLE modem sleep: use main crystal as LP clock (no ext 32kHz needed)
            ("# CONFIG_BT_CTRL_MODEM_SLEEP is not set",
             "CONFIG_BT_CTRL_MODEM_SLEEP=y\n"
             "CONFIG_BT_CTRL_MODEM_SLEEP_MODE_1=y\n"
             "CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL=y\n"
             "CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP=y"),
            # Power management + tickless idle (required for automatic light sleep)
            ("# CONFIG_PM_ENABLE is not set",
             "CONFIG_PM_ENABLE=y"),
            ("# CONFIG_FREERTOS_USE_TICKLESS_IDLE is not set",
             "CONFIG_FREERTOS_USE_TICKLESS_IDLE=y\n"
             "CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP=3"),
        ]:
            if opt[0] in cfg:
                cfg = cfg.replace(opt[0], opt[1])
                patched = True

        # Fix BT_CTRL_SLEEP derived values (kconfig doesn't auto-resolve these)
        if "CONFIG_BT_CTRL_MODEM_SLEEP=y" in cfg:
            if "CONFIG_BT_CTRL_SLEEP_MODE_EFF=0" in cfg:
                cfg = cfg.replace("CONFIG_BT_CTRL_SLEEP_MODE_EFF=0",
                                  "CONFIG_BT_CTRL_SLEEP_MODE_EFF=1")
                patched = True
            if "CONFIG_BT_CTRL_SLEEP_CLOCK_EFF=0" in cfg:
                cfg = cfg.replace("CONFIG_BT_CTRL_SLEEP_CLOCK_EFF=0",
                                  "CONFIG_BT_CTRL_SLEEP_CLOCK_EFF=1")
                patched = True

        # Add SPIRAM_BOOT_INIT + IGNORE_NOTFOUND after SPIRAM_BOOT_HW_INIT if missing
        if "CONFIG_SPIRAM_BOOT_INIT=y" not in cfg and "CONFIG_SPIRAM_BOOT_HW_INIT=y" in cfg:
            cfg = cfg.replace(
                "CONFIG_SPIRAM_BOOT_HW_INIT=y",
                "CONFIG_SPIRAM_BOOT_HW_INIT=y\nCONFIG_SPIRAM_BOOT_INIT=y\nCONFIG_SPIRAM_IGNORE_NOTFOUND=y")
            patched = True

        if patched:
            with open(base_sdkconfig, "w") as f:
                f.write(cfg)
            print("[patch_sections_ld] patched base sdkconfig at %s" % base_sdkconfig)

    # ── 3.  Patch sdkconfig.h (HybridCompile kconfig merge misses some settings)
    sdkconfig_h = os.path.join(libs_dir, mcu, mem_type, "include", "sdkconfig.h")
    if os.path.isfile(sdkconfig_h):
        with open(sdkconfig_h, "r") as f:
            hdr = f.read()

        hdr_patched = False

        # BLE modem sleep defines that kconfig merge fails to propagate
        bt_defines = [
            ("#define CONFIG_BT_CTRL_SLEEP_MODE_EFF 0",
             "#define CONFIG_BT_CTRL_SLEEP_MODE_EFF 1"),
            ("#define CONFIG_BT_CTRL_SLEEP_CLOCK_EFF 0",
             "#define CONFIG_BT_CTRL_SLEEP_CLOCK_EFF 1"),
        ]
        for old, new in bt_defines:
            if old in hdr:
                hdr = hdr.replace(old, new)
                hdr_patched = True

        # PM + tickless idle defines (critical for light sleep)
        pm_defines = {
            "CONFIG_PM_ENABLE": "#define CONFIG_PM_ENABLE 1",
            "CONFIG_FREERTOS_USE_TICKLESS_IDLE": "#define CONFIG_FREERTOS_USE_TICKLESS_IDLE 1",
            "CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP": "#define CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP 3",
        }
        for name, define_line in pm_defines.items():
            if ("#define %s " % name) not in hdr:
                # Add before the end-of-file guard or at end
                hdr = hdr.rstrip() + "\n" + define_line + "\n"
                hdr_patched = True
                print("[patch_sections_ld] added %s to sdkconfig.h" % name)

        # Add missing BLE modem sleep defines if not present
        if "#define CONFIG_BT_CTRL_MODEM_SLEEP " not in hdr:
            # Insert after the last BT_CTRL define
            insert_after = "#define CONFIG_BT_CTRL_SLEEP_CLOCK_EFF"
            idx = hdr.find(insert_after)
            if idx >= 0:
                end_of_line = hdr.index("\n", idx)
                bt_sleep_defines = (
                    "\n#define CONFIG_BT_CTRL_MODEM_SLEEP 1"
                    "\n#define CONFIG_BT_CTRL_MODEM_SLEEP_MODE_1 1"
                    "\n#define CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL 1"
                    "\n#define CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP 1"
                )
                hdr = hdr[:end_of_line] + bt_sleep_defines + hdr[end_of_line:]
                hdr_patched = True

        if hdr_patched:
            with open(sdkconfig_h, "w") as f:
                f.write(hdr)
            print("[patch_sections_ld] patched sdkconfig.h at %s" % sdkconfig_h)

    # ── 4.  Patch .dummy/CMakeLists.txt to include BT component ─────────
    #   HybridCompile's dummy project has no BT dependency, so CMake
    #   excludes the BT component and kconfig ignores all BT options.
    #   Adding REQUIRES bt ensures BT libs are built with our modem sleep settings.
    dummy_cmake = os.path.join(env.get("PROJECT_DIR", "."), ".dummy", "CMakeLists.txt")
    if os.path.isfile(dummy_cmake):
        with open(dummy_cmake, "r") as f:
            cmake_txt = f.read()
        if "REQUIRES" not in cmake_txt and "idf_component_register" in cmake_txt:
            cmake_txt = cmake_txt.replace(
                'INCLUDE_DIRS ".")',
                'INCLUDE_DIRS "." REQUIRES bt esp_pm)')
            with open(dummy_cmake, "w") as f:
                f.write(cmake_txt)
            print("[patch_sections_ld] patched .dummy/CMakeLists.txt to include bt + esp_pm")
