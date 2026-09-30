#!/bin/bash
# BPI-R4 firmware builder: thermal mod (PWM fan) + optional SFP ONU recovery console.
#   RECOVERY_UART=1 ./build_r4_firmware.sh   (default) - recovery image
#   RECOVERY_UART=0 ./build_r4_firmware.sh             - thermal mod only
set -e

OPENWRT_VERSION="${OPENWRT_VERSION:-25.12.4}"
RECOVERY_UART="${RECOVERY_UART:-1}"
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORK_DIR="$SCRIPT_DIR/bpi_r4_factory"
RECOVERY_SRC="$SCRIPT_DIR/recovery"
THREADS=$(( $(nproc) - 1 ))
[ "$THREADS" -lt 1 ] && THREADS=1

mkdir -p "$WORK_DIR"
cd "$WORK_DIR"

# IB_ONLY=1 — пропустить шаги 1-3 (исходники/ядро уже собраны), пересобрать только образ в Image Builder
if [ "$IB_ONLY" = "1" ]; then
    echo "=== IB_ONLY: шаги 1-3 пропущены, используем готовый build_dir ==="
    cd openwrt-source
else
echo "=== 1. Исходники OpenWrt v$OPENWRT_VERSION ==="
if [ ! -d "openwrt-source" ]; then
    git clone --depth 1 --branch "v$OPENWRT_VERSION" https://git.openwrt.org/openwrt/openwrt.git openwrt-source
else
    cd openwrt-source
    CURRENT_TAG=$(git describe --tags --always 2>/dev/null || echo "")
    if [ "$CURRENT_TAG" != "v$OPENWRT_VERSION" ]; then
        echo "Текущая версия ($CURRENT_TAG) отличается от запрашиваемой (v$OPENWRT_VERSION). Перекачиваем..."
        cd ..
        rm -rf openwrt-source
        git clone --depth 1 --branch "v$OPENWRT_VERSION" https://git.openwrt.org/openwrt/openwrt.git openwrt-source
    else
        cd ..
    fi
fi
cd openwrt-source

./scripts/feeds update -a
./scripts/feeds install -a

cat <<EOF > .config
CONFIG_TARGET_mediatek=y
CONFIG_TARGET_mediatek_filogic=y
CONFIG_TARGET_mediatek_filogic_DEVICE_bananapi_bpi-r4=y
CONFIG_KERNEL_DEVMEM=y
EOF
make defconfig

echo "=== 2. Патчинг ==="
git checkout -- target/linux/mediatek/filogic/config-6.12

if [ "$RECOVERY_UART" = "1" ]; then
    echo "--- Recovery UART: /dev/mem без ограничений (MMIO доступ к GPIO из gpiouart)"
    echo "CONFIG_DEVMEM=y" >> target/linux/mediatek/filogic/config-6.12
    echo "# CONFIG_STRICT_DEVMEM is not set" >> target/linux/mediatek/filogic/config-6.12
fi

# Чистое ядро -> гарантированно чистый DTS перед дописыванием
make target/linux/clean
make target/linux/prepare -j"$THREADS"

DTS_FILE=$(find build_dir/target-*/linux-mediatek_filogic/linux-*/arch/arm64/boot/dts/mediatek/ -name "mt7988a-bananapi-bpi-r4.dts" -print -quit)
[ -n "$DTS_FILE" ] || { echo "DTS не найден"; exit 1; }

echo "--- Thermal mod (4 уровня, 50-65°C) -> $DTS_FILE"
cat << 'EOF' >> "$DTS_FILE"

// === BPI-R4 THERMAL MOD START ===
&pio {
        pwm6_pins: pwm6-pins {
                mux { function = "pwm"; groups = "pwm6_0"; };
        };
};

&pwm {
        pinctrl-names = "default";
        pinctrl-0 = <&pwm6_pins>;
        status = "okay";
};

/ {
        sfp_fan: sfp-fan {
                compatible = "pwm-fan";
                pwms = <&pwm 6 50000>;
                cooling-levels = <0 75 125 180 255>;
                #cooling-cells = <2>;
        };
};

&{/thermal-zones} {
        wifi-sfp-thermal {
                polling-delay-passive = <1000>;
                polling-delay = <5000>;
                thermal-sensors = <&lvts 2>;

                trips {
                        sfp_t0: sfp-t0 { temperature = <50000>; hysteresis = <2000>; type = "active"; };
                        sfp_t1: sfp-t1 { temperature = <55000>; hysteresis = <2000>; type = "active"; };
                        sfp_t2: sfp-t2 { temperature = <60000>; hysteresis = <2000>; type = "active"; };
                        sfp_t3: sfp-t3 { temperature = <65000>; hysteresis = <2000>; type = "active"; };
                };

                cooling-maps {
                        map0 { trip = <&sfp_t0>; cooling-device = <&sfp_fan 1 1>; };
                        map1 { trip = <&sfp_t1>; cooling-device = <&sfp_fan 2 2>; };
                        map2 { trip = <&sfp_t2>; cooling-device = <&sfp_fan 3 3>; };
                        map3 { trip = <&sfp_t3>; cooling-device = <&sfp_fan 4 4>; };
                };
        };
};
// === BPI-R4 THERMAL MOD END ===
EOF

# CONSOLE_PINS=1 (thermal image): free the four SFP1 control GPIOs so the module console works from the
# normal firmware too; the sfp driver keeps mod-def0 + i2c and still switches to 2500base-x.
if [ "$CONSOLE_PINS" = "1" ] && [ "$RECOVERY_UART" != "1" ]; then
    cat << 'EOF' >> "$DTS_FILE"

// === CONSOLE PINS MOD START ===
&sfp1 {
        /delete-property/ tx-fault-gpios;
        /delete-property/ rate-select0-gpios;
        /delete-property/ tx-disable-gpios;
        /delete-property/ los-gpios;
};
// === CONSOLE PINS MOD END ===
EOF
fi

if [ "$RECOVERY_UART" = "1" ]; then
    echo "--- Recovery UART DTS: освобождаем пины SFP1 (pio21/69/70) от драйвера sfp, изолируем CPU1/2 под bit-bang"
    # sfp2 тоже выключен: его tx-disable/rate-select сидят в том же банке DOUT (пины 0-3),
    # что и наш TX (pio21) — исключаем RMW-гонку ядра с gpiouart.
    # Аппаратный UART2 на пинах клетки НЕ используется: по pinctrl-mt7988.c pio1 = UART2_TXD,
    # а это линия, где передаёт сам модуль. Только bit-bang.
    cat << 'EOF' >> "$DTS_FILE"

// === RECOVERY UART MOD START ===
// sfp1 stays ENABLED so phylink switches sfp-wan to 2500base-x and the module's host link can be
// tested from this image; only the four control GPIOs go to gpiouart (mod-def0 + i2c remain).
&sfp1 {
        /delete-property/ tx-fault-gpios;
        /delete-property/ rate-select0-gpios;
        /delete-property/ tx-disable-gpios;
        /delete-property/ los-gpios;
};

&sfp2 {
        status = "disabled";
};

/ {
        chosen {
                bootargs-append = " isolcpus=1,2 irqaffinity=0,3";
        };
};
// === RECOVERY UART MOD END ===
EOF
fi

echo "=== 3. make -j$THREADS ==="
make -j"$THREADS"
fi   # IB_ONLY

if [ "$RECOVERY_UART" = "1" ] || [ "$CONSOLE_PINS" = "1" ]; then
    echo "=== 3a. gpiouart (cross) ==="
    CROSS_GCC=$(find staging_dir/toolchain-aarch64_cortex-a53_gcc-* -name "aarch64-openwrt-linux-gcc" -print -quit)
    [ -n "$CROSS_GCC" ] || { echo "Кросс-компилятор не найден"; exit 1; }
    "$CROSS_GCC" -O2 -Wall -pthread -o "$WORK_DIR/gpiouart" "$RECOVERY_SRC/gpiouart.c"
    file "$WORK_DIR/gpiouart" || true
fi

GOLDEN_DTB=$(realpath "$(find build_dir/target-*/linux-mediatek_filogic/linux-*/arch/arm64/boot/dts/mediatek/ -name "mt7988a-bananapi-bpi-r4.dtb" -print -quit)")
[ -f "$GOLDEN_DTB" ] || { echo "DTB не собрался"; exit 1; }

echo "=== 4. Image Builder ==="
cd "$WORK_DIR"
IB_URL="https://downloads.openwrt.org/releases/$OPENWRT_VERSION/targets/mediatek/filogic/openwrt-imagebuilder-$OPENWRT_VERSION-mediatek-filogic.Linux-x86_64.tar.zst"
IB_DIR="openwrt-imagebuilder-$OPENWRT_VERSION-mediatek-filogic.Linux-x86_64"
if [ ! -d "$IB_DIR" ]; then
    [ -f "$(basename "$IB_URL")" ] || wget "$IB_URL"
    tar --zstd -xf "$(basename "$IB_URL")"
fi
cd "$IB_DIR"

echo "=== 5. DTB и ядро в Image Builder ==="
TARGET_DIR="build_dir/target-aarch64_cortex-a53_musl/linux-mediatek_filogic"
mkdir -p "$TARGET_DIR"
cp "$GOLDEN_DTB" "$TARGET_DIR/image-mt7988a-bananapi-bpi-r4.dtb"

SRC_KERNEL_DIR="../openwrt-source/build_dir/target-aarch64_cortex-a53_musl/linux-mediatek_filogic"
if [ -f "$SRC_KERNEL_DIR/bananapi_bpi-r4-kernel.bin" ]; then
    echo "Ядро из своей сборки (DEVMEM) -> Image Builder"
    cp "$SRC_KERNEL_DIR/bananapi_bpi-r4-kernel.bin" "$TARGET_DIR/"
    cp "$SRC_KERNEL_DIR/vmlinux" "$TARGET_DIR/" 2>/dev/null || true
    # КРИТИЧНО: IB берёт vermagic из $(LINUX_DIR)/.vermagic, а без него подставляет хеш
    # официального ядра (include/kernel.mk:49-50) — тогда в образ попадают официальные kmod,
    # которые не грузятся на нашем ядре ("section size must match the kernel's built struct module").
    VERMAGIC=$(cat "$SRC_KERNEL_DIR"/linux-*/.vermagic)
    LINUX_SUBDIR=$(basename "$(ls -d "$SRC_KERNEL_DIR"/linux-*/ | head -1)")
    mkdir -p "$TARGET_DIR/$LINUX_SUBDIR"
    echo "$VERMAGIC" > "$TARGET_DIR/$LINUX_SUBDIR/.vermagic"
    echo "vermagic своего ядра: $VERMAGIC -> $TARGET_DIR/$LINUX_SUBDIR/.vermagic"
else
    echo "ПРЕДУПРЕЖДЕНИЕ: bananapi_bpi-r4-kernel.bin не найден, останется ядро Image Builder"
fi

echo "=== 6. Образ ==="
rm -rf files; mkdir -p files
if [ "$CONSOLE_PINS" = "1" ] && [ "$RECOVERY_UART" != "1" ]; then
    echo "--- gpiouart + командные файлы (без автозапуска сервиса)"
    install -D -m 0755 "$WORK_DIR/gpiouart"              files/usr/sbin/gpiouart
    install -D -m 0644 "$RECOVERY_SRC/cmds_uboot.txt"    files/root/recovery/cmds_uboot.txt
    install -D -m 0644 "$RECOVERY_SRC/cmds_linux.txt"    files/root/recovery/cmds_linux.txt
    install -D -m 0644 "$RECOVERY_SRC/cmds_failsafe.txt" files/root/recovery/cmds_failsafe.txt
    install -D -m 0644 "$RECOVERY_SRC/README.txt"        files/root/recovery/README.txt
    mkdir -p files/etc
    echo "thermal+console $(date '+%Y-%m-%d %H:%M') gpiouart:$(md5sum "$RECOVERY_SRC/gpiouart.c" | cut -c1-8)" > files/etc/sfp-recovery.buildinfo
fi
EXTRA_PACKAGES=""
SFP_PACKAGE="kmod-sfp"
if [ "$RECOVERY_UART" = "1" ]; then
    SFP_PACKAGE=""
    BUILD_ID="recovery $(date '+%Y-%m-%d %H:%M') gpiouart:$(md5sum "$RECOVERY_SRC/gpiouart.c" | cut -c1-8) owrt:$OPENWRT_VERSION"
    echo "--- Файлы recovery ($BUILD_ID)"
    install -D -m 0755 "$WORK_DIR/gpiouart"              files/usr/sbin/gpiouart
    install -D -m 0755 "$RECOVERY_SRC/sfp-recovery.init" files/etc/init.d/sfp-recovery
    install -D -m 0755 "$RECOVERY_SRC/99-sfp-recovery"   files/etc/uci-defaults/99-sfp-recovery
    # rc.d-симлинки кладём в образ явно: uci-defaults выполняется уже после того, как procd
    # просканировал /etc/rc.d, и иначе сервис стартовал бы только со второй загрузки
    mkdir -p files/etc/rc.d
    ln -sf ../init.d/sfp-recovery files/etc/rc.d/S99sfp-recovery
    ln -sf ../init.d/sfp-recovery files/etc/rc.d/K10sfp-recovery
    install -D -m 0644 "$RECOVERY_SRC/cmds_uboot.txt"    files/root/recovery/cmds_uboot.txt
    install -D -m 0644 "$RECOVERY_SRC/cmds_linux.txt"    files/root/recovery/cmds_linux.txt
    install -D -m 0644 "$RECOVERY_SRC/cmds_failsafe.txt" files/root/recovery/cmds_failsafe.txt
    install -D -m 0644 "$RECOVERY_SRC/README.txt"        files/root/recovery/README.txt
    echo "$BUILD_ID" > files/etc/sfp-recovery.buildinfo
    install -D -m 0644 "$RECOVERY_SRC/gpiouart.c"        files/usr/share/sfp-recovery/gpiouart.c

    echo "--- Локальный репозиторий kmod (vermagic своего ядра)"
    rm -f packages/kernel-*.apk packages/kmod-*.apk            # старые хеши от прошлых сборок
    cp "$WORK_DIR/openwrt-source/bin/targets/mediatek/filogic/packages/"*.apk packages/
    ./staging_dir/host/bin/apk mkndx --allow-untrusted -o packages/packages.adb packages/*.apk
    LOCAL_REPO="file://$WORK_DIR/$IB_DIR/packages/packages.adb"
    grep -v '^file://' repositories > repositories.tmp || true
    { echo "$LOCAL_REPO"; cat repositories.tmp; } > repositories
    rm -f repositories.tmp
    sed -i 's/CONFIG_SIGNATURE_CHECK=y/# CONFIG_SIGNATURE_CHECK is not set/' .config 2>/dev/null || true
fi

make image PROFILE="bananapi_bpi-r4" FILES="files" \
    PACKAGES="apk-mbedtls base-files ca-bundle dnsmasq dropbear firewall4 fitblk fstools kmod-crypto-hw-safexcel kmod-gpio-button-hotplug kmod-leds-gpio kmod-nft-offload libc libgcc libustream-mbedtls logd mtd netifd nftables odhcp6c odhcpd-ipv6only ppp ppp-mod-pppoe procd-ujail uboot-envtools uci uclient-fetch urandom-seed urngd wpad-basic-mbedtls kmod-hwmon-pwmfan kmod-i2c-mux-pca954x kmod-eeprom-at24 kmod-mt7996-firmware kmod-mt7996-233-firmware kmod-rtc-pcf8563 ${SFP_PACKAGE} kmod-usb3 e2fsprogs f2fsck mkf2fs mt7988-wo-firmware luci${EXTRA_PACKAGES}"

echo "=== 7. Результат ==="
OUT="$WORK_DIR/out"; mkdir -p "$OUT"
STAMP=$(date '+%Y%m%d-%H%M')
TAG=thermal; [ "$CONSOLE_PINS" = "1" ] && TAG=thermal-console; [ "$RECOVERY_UART" = "1" ] && TAG=recovery
for f in bin/targets/mediatek/filogic/*bananapi_bpi-r4-sdcard.img.gz bin/targets/mediatek/filogic/*bananapi_bpi-r4-squashfs-sysupgrade.itb; do
    [ -f "$f" ] && cp "$f" "$OUT/BPi-R4-$TAG-$STAMP-$(basename "$f" | sed 's/openwrt-[0-9.]*-mediatek-filogic-bananapi_bpi-r4-//')"
done
ls -la "$OUT"
echo "DONE ($TAG). Образы в $OUT"
