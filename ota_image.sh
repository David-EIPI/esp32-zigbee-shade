build_name=`sed -n "/^project/s,project(\(.*\)),\1,p" CMakeLists.txt`

img_version=`awk '/OTA_UPGRADE_RUNNING_FILE_VERSION/{print $3}' main/ota.h`
manufacturer_id=`awk '/OTA_UPGRADE_MANUFACTURER/{print $3}' main/ota.h`

release_notes="Removed OTA rollback support."

/usr/bin/python ~/esp/esp-zigbee-sdk/tools/image_builder_tool/image_builder_tool.py \
    --create ota/${build_name}.ota.bin \
    --manuf-id $manufacturer_id \
    --image-type 0x1011 \
    --version ${img_version} \
    --tag-id 0x0000 \
    --min-hw-ver 0x0101 \
    --max-hw-ver 0x0101 \
    --tag-file build/${build_name}.bin

file_sz=`stat -c %s ota/${build_name}.ota.bin`
checksum=(`openssl dgst -sha3-256 ota/${build_name}.ota.bin`)
curdate=`date +"%F %T"`

cat >ota/local_index.json <<EOJ
// Place this and binary files in HA configured OTA directory, e.g. /config/zha_ota
// Example configuration entry in configuration.yaml:
// zha:
//   ota:
//     extra_providers:
//       - type: zigpy_local
//         index_file: /config/zha_ota/local_index.json
{
    "firmwares": [
        {
            "path": "shade-zigbee.ota.bin",
            "file_version": ${img_version},
            "file_size": $file_sz,
            "image_type": 4113,
            "manufacturer_names": ["DS"],
            "model_names": ["Shade1"],
            "manufacturer_id": $((0+manufacturer_id)),
            "changelog": "${release_notes}",
            "release_notes": "$curdate: ${release_notes}",
            "checksum": "sha3-256:${checksum[1]}",
            "min_hardware_version": 257,
            "max_hardware_version": 257,
            "min_current_file_version": 0,
            "max_current_file_version": ${img_version},
            "specificity": 999999
        }
    ]
}
EOJ
