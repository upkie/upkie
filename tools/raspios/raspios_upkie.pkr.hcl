packer {
  required_plugins {
    git = {
      version = ">=v0.3.2"
      source  = "github.com/ethanmdavidson/git"
    }
  }
}

source "arm" "raspios_oldstable_arm64" {
  file_urls             = ["https://downloads.raspberrypi.com/raspios_oldstable_arm64/images/raspios_oldstable_arm64-2024-03-12/2024-03-12-raspios-bullseye-arm64.img.xz"]
  file_checksum_url     = "https://downloads.raspberrypi.com/raspios_oldstable_arm64/images/raspios_oldstable_arm64-2024-03-12/2024-03-12-raspios-bullseye-arm64.img.xz.sha256"
  file_checksum_type    = "sha256"
  file_target_extension = "xz"
  file_unarchive_cmd    = ["xz", "--decompress", "$ARCHIVE_PATH"]
  image_build_method    = "resize"
  image_path            = "raspios_upkie.img"
  image_size            = "6G"
  image_type            = "dos"
  image_partitions {
    name         = "boot"
    type         = "c"
    start_sector = "2048"
    filesystem   = "fat"
    size         = "256M"
    mountpoint   = "/boot/firmware"
  }
  image_partitions {
    name         = "root"
    type         = "83"
    start_sector = "526336"
    filesystem   = "ext4"
    size         = "0"
    mountpoint   = "/"
  }
  image_chroot_env             = ["PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin"]
  qemu_binary_source_path      = "/usr/bin/qemu-aarch64-static"
  qemu_binary_destination_path = "/usr/bin/qemu-aarch64-static"
}

build {
  sources = ["source.arm.raspios_oldstable_arm64"]

  provisioner "file" {
    source = "local/configure_cpu_isolation.py"
    destination = "/root/configure_cpu_isolation.py"
  }

  provisioner "file" {
    source = "local/hard_rezero"
    destination = "/usr/local/bin/hard_rezero"
  }

  provisioner "file" {
    source = "local/micromamba"
    destination = "/usr/local/bin/micromamba"
  }

  provisioner "file" {
    source = "local/pi3hat_spine"
    destination = "/usr/local/bin/pi3hat_spine"
  }

  provisioner "file" {
    source = "local/stop_servos"
    destination = "/usr/local/bin/stop_servos"
  }

  provisioner "file" {
    source = "local/upkie_tool"
    destination = "/usr/local/bin/upkie_tool"
  }

  provisioner "shell" {
    script = "provision.sh"
  }
}
