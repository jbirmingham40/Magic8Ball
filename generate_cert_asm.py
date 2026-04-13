"""
PlatformIO pre-build script — generate .S assembly files to embed
certificate data required by esp_insights and esp_rainmaker components.

Mimics ESP-IDF's target_add_binary_data() CMake function.
"""
import os

Import("env")

project_dir = env.get("PROJECT_DIR", ".")
build_dir = env.subst("$BUILD_DIR")

# All certs referenced by target_add_binary_data in managed_components
certs = [
    ("managed_components/espressif__esp_insights/server_certs/mqtt_server.crt", "mqtt_server.crt"),
    ("managed_components/espressif__esp_insights/server_certs/https_server.crt", "https_server.crt"),
    ("managed_components/espressif__esp_rainmaker/server_certs/rmaker_mqtt_server.crt", "rmaker_mqtt_server.crt"),
    ("managed_components/espressif__esp_rainmaker/server_certs/rmaker_claim_service_server.crt", "rmaker_claim_service_server.crt"),
    ("managed_components/espressif__esp_rainmaker/server_certs/rmaker_ota_server.crt", "rmaker_ota_server.crt"),
]

os.makedirs(build_dir, exist_ok=True)

for rel_path, filename in certs:
    cert_path = os.path.join(project_dir, rel_path)
    asm_path = os.path.join(build_dir, filename + ".S")

    if not os.path.isfile(cert_path):
        print("[generate_cert_asm] WARNING: %s not found — skipping" % rel_path)
        continue

    # Symbol name: _binary_<filename_with_underscores>
    sym = "_binary_" + filename.replace(".", "_").replace("-", "_")

    lines = [
        '.section .rodata.embedded',
        '.global %s_start' % sym,
        '.type %s_start, @object' % sym,
        '.align 4',
        '%s_start:' % sym,
        '.incbin "%s"' % cert_path.replace("\\", "/"),
        '.global %s_end' % sym,
        '%s_end:' % sym,
        '.byte 0',
        '',
    ]

    with open(asm_path, "w") as f:
        f.write("\n".join(lines))

print("[generate_cert_asm] generated %d cert .S files in %s" % (len(certs), build_dir))
