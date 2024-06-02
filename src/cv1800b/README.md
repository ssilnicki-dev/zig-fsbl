In order to use compiled milk-v-duo-bl33.bin with Milk-V Duo:

1. clone & compile https://github.com/milkv-duo/duo-buildroot-sdk
2. copy zig-out/bin/milk-v-duo-bl33.bin into duo-buildroot-sdk/fsbl
3. cd into duo-buildroot-sdk/fsbl
4. run
```
./plat/cv180x/fiptool.py -v genfip \
'build/cv1800b_milkv_duo_sd/fip.bin' \
--MONITOR_RUNADDR="0x0000000080000000" \
--CHIP_CONF='build/cv1800b_milkv_duo_sd/chip_conf.bin' \
--NOR_INFO='FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF' \
--NAND_INFO='00000000'\
--BL2='build/cv1800b_milkv_duo_sd/bl2.bin' \
--DDR_PARAM='test/cv181x/ddr_param.bin' \
--MONITOR='../opensbi/build/platform/generic/firmware/fw_dynamic.bin' \
--LOADER_2ND='milk-v-duo-bl33.bin' \
--compress='lzma'
```
5. copy fip.bin into first FAT32 partition of SD card, prepared for Milk-V Duo board
