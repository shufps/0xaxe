# 0xaxe

0xAxe is a 16-ASIC-BM1366 Miner based on the [QAxe](https://github.com/shufps/qaxe)

<img src="https://github.com/shufps/0xaxe/assets/3079832/f278c0d4-3be7-44aa-b233-c5c798653ac6" width="600px">


**rev1**: not tested</br>
**rev1.1**: working but needs some manual fixes</br>
**rev1.2**: should be working without fixes, but not tested yet.</br>

0xAxe is working (see screenshot at the end of the page to get an impression what to expect) 🥳

**note**: The 0xAxe is not a stand-alone device because it only supports USB but it can be run connected to a Raspberry Pi without problems. Also multiple devices can be connected to a single Pi. 

ASICs
=====

The 0xAxe uses 16 ASICs of type BM1366

![image](https://github.com/shufps/0xaxe/assets/3079832/0f3c1088-be82-4bf8-898d-34b336d1b7bd)

Compilation (Bootloader or CMSIS-DAP)
======================================

```bash
# install curl
sudo apt install curl

# install rust
curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh

# add to ~/.bash.rc (afterwards, opening a new terminal is needed)
echo 'source "$HOME/.cargo/env"' >> ~/.bashrc

# clone repository
git clone https://github.com/shufps/0xaxe

# clone submodules
cd 0xaxe
git submodule init
git submodule update

# add rust target for STM32L0 variants
rustup target add thumbv6m-none-eabi

# build firmware for L072
cd firmware/fw-L072KZ
./build.sh
```

Installation via USB Bootloader on board with `BOOT` button
===========================================================
The STM32L072CB variant has an integrated DFU Bootloader that starts when pressing the `BOOT` button during reset.

Afterwards the firmware can be flashed via `dfu-utils`:

```bash
# install cargo-binutils and llvm tools
cargo install cargo-binutils
rustup component add llvm-tools-preview

# create the firmware.bin
DEFMT_LOG=info cargo objcopy --release --bin qaxe -- -O binary 0xaxe.bin

# install dfu-utils
sudo apt-get install dfu-util

now start the stm32 in DFU mode by pressing `boot` 

# after booting, list the devices
dfu-util --list

# flash the binary
dfu-util -a 0 -s 0x08000000:leave -D 0xaxe.bin
```


Installation via CMSIS-DAP Programmer
=====================================

**note**: Using CMSIS-DAP and PicoProbe is only interesting for developers trying to alter the firmware.

As programming/debug adapter the Picoprobe firmware running on a Raspi Pico works best: <br>
https://github.com/rp-rs/rp2040-project-template/blob/main/debug_probes.md / https://github.com/raspberrypi/picoprobe/releases/tag/picoprobe-cmsis-v1.0.3
<br>
<br>
There also is a little board with only 3 parts that gives a nice low-cost solution to flash the Qaxe:<br>
https://github.com/shufps/raspi-pico-dap

On `rev3` there should be the option to boot the stm32 (by pressing the `boot`-button on reset) into DFU-Bootloader mode what makes flashing via USB and without CMSIS-DAP programmer possible.

## Flashing

After the source was compiled it is flashed by:

```bash
# build firmware for L072
cd firmware/fw-L072KZ
# run firmware (this also flashes it to the stm32)
./run.sh
```

Mining Client
=============

![image](https://github.com/shufps/0xaxe/assets/3079832/8c144fcf-1d3e-4634-a884-1094abb9330f)


Misc
====
If you like this project and want to support future work, feel free to donate to: `bc1q29hp4fqtks2wzpmfwtpac64fnr8ujw2nvnra04`



Stratum Mining Client:<br>
https://github.com/shufps/piaxe-miner
