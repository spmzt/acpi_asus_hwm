# acpi_asus_hwm

FreeBSD kernel module for fan monitoring and control on ASUS desktop
motherboards with a Nuvoton NCT6791D–NCT6799D Super I/O chip.

The firmware owns the Super I/O ports on these boards, but exposes the
hardware monitor through the `WMBD` method of the WMI device whose `_UID` is
`AsusMbSwInterface` (or `ASUSWMI` on older boards).  The driver performs all
register accesses through that method.

Each fan is available through `sysctl(8)` (`dev.acpi_asus_hwm.0.fanN.rpm`,
`.manual`, `.pwm`) and as a `pwmc(4)` channel, so `pwm(8)` and
`sysutils/fand` can drive it.  See `acpi_asus_hwm(4)`.

## Build and load

    make
    kldload ./acpi_asus_hwm.ko

Building needs the kernel sources in `/usr/src` (or set `SYSDIR`).

## Examples

Fan `N` is channel `N-1` of `pwmc0`, so fan 2 is `/dev/pwm/pwmc0.1`.

With `pwm(8)`, run fan 2 at 60% and then hand it back to the firmware
curve:

    pwm -f pwmc0.1 -E -p 40000 -d 60%
    pwm -f pwmc0.1 -D

Always pass a period: the driver rejects a period of 0, which would turn
every percentage into a duty cycle of 0.

With `sysutils/fand`, drive fan 2 from the temperature of CPU 0: 40% from
50°C, full speed from 75°C, and back to the firmware curve below 45°C.
In `/etc/rc.conf`:

    fand_enable="YES"
    fand_sensor="dev.cpu.0.temperature"
    fand_device="pwmc0.1"
    fand_temp="50"
    fand_off_temp="45"
    fand_period="40000"
    fand_duty="40%"
    fand_multiplier="0.1"
    fand_flags="-T 75 -P 40000 -D 100% -i 5"

`fand_multiplier` converts the tenths of a kelvin reported by
`coretemp(4)`.  `fand` does not hand the fan back when it stops; run
`pwm -f pwmc0.1 -D` afterwards.

## Warning

Under manual control a fan ignores temperature.  Never stop the CPU fan or
the pump of a liquid cooler: the processor can overheat within seconds.
