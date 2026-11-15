/*
 * Copyright (c) 2026 Pouria Mousavizadeh Tehrani <pouria@FreeBSD.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * Fan monitoring and control for ASUS desktop motherboards with a Nuvoton
 * NCT679x Super I/O chip.
 *
 * The firmware claims the Super I/O ports, but exposes them through the
 * "WMBD" method of a WMI device whose _UID is "AsusMbSwInterface" (or
 * "ASUSWMI" on older boards).  WMBD is not listed in _WDG, so it is
 * evaluated directly rather than through acpi_wmi(4).
 *
 * Each fan is also a pwmbus(9) channel, so pwm(8) and pwmc(4) consumers
 * can drive it.  Enabling a channel puts the fan under manual control;
 * disabling it hands the fan back to the firmware curve.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/pwm/pwmbus.h>
#include <dev/pwm/pwmc.h>

#include "pwmbus_if.h"

#define _COMPONENT ACPI_OEM
ACPI_MODULE_NAME("asus_hwm")

ACPI_SERIAL_DECL(asus_hwm, "ASUS hardware monitor");

#define	ASUS_HWM_METHOD			"WMBD"
#define	ASUS_HWM_UID_MSI		"AsusMbSwInterface"
#define	ASUS_HWM_UID_WMI		"ASUSWMI"

/* WMBD method IDs */
#define	ASUS_HWM_METHODID_RSIO		0x5253494F	/* Read Super I/O */
#define	ASUS_HWM_METHODID_WSIO		0x5753494F	/* Write Super I/O */
#define	ASUS_HWM_METHODID_RHWM		0x5248574D	/* Read HW monitor */
#define	ASUS_HWM_METHODID_WHWM		0x5748574D	/* Write HW monitor */

/* Super I/O configuration registers */
#define	NCT_LD_HWM			0x0b
#define	NCT_SIO_REG_DEVID		0x20
#define	NCT_SIO_ID_MASK			0xfff8

/* Hardware monitor registers (bank << 8 | index) */
#define	NCT_REG_CHIPID			0x058
#define	NCT_CHIPID			0xc1
#define	NCT_FAN_MODE_SHIFT		4
#define	NCT_FAN_MODE_MASK		0xf0
#define	NCT_FAN_MODE_MANUAL		0

#define	ASUS_HWM_MAXFAN			7
#define	ASUS_HWM_PWM_PERIOD		40000	/* ns, 25 kHz */

static const uint16_t nct_reg_fan[ASUS_HWM_MAXFAN] = {
	0x4c0, 0x4c2, 0x4c4, 0x4c6, 0x4c8, 0x4ca, 0x4ce
};
static const uint16_t nct_reg_pwm[ASUS_HWM_MAXFAN] = {
	0x109, 0x209, 0x309, 0x809, 0x909, 0xa09, 0xb09
};
static const uint16_t nct_reg_pwm_read[ASUS_HWM_MAXFAN] = {
	0x001, 0x003, 0x011, 0x013, 0x015, 0xa09, 0xb09
};
static const uint16_t nct_reg_fan_mode[ASUS_HWM_MAXFAN] = {
	0x102, 0x202, 0x302, 0x802, 0x902, 0xa02, 0xb02
};

struct nct_chip {
	uint16_t	id;
	const char	*name;
	int		nfan;
};

static const struct nct_chip nct_chips[] = {
	{ 0xc800, "NCT6791D", 6 },
	{ 0xc910, "NCT6792D", 6 },
	{ 0xd120, "NCT6793D", 6 },
	{ 0xd350, "NCT6795D", 6 },
	{ 0xd420, "NCT6796D", 7 },
	{ 0xd450, "NCT6797D", 7 },
	{ 0xd428, "NCT6798D", 7 },
	{ 0xd800, "NCT6799D", 7 },
	{ 0, NULL, 0 }
};

struct asus_hwm_fan {
	uint8_t	saved_mode;	/* firmware fan mode register */
	uint8_t	saved_pwm;	/* firmware duty register */
	bool	manual;		/* we put this fan into manual mode */
	u_int	period;		/* pwmbus period, in ns */
	uint8_t	duty;		/* pwmbus duty, applied on enable */
	bool	duty_set;	/* duty was configured through pwmbus */
};

struct acpi_asus_hwm_softc {
	device_t	dev;
	device_t	busdev;
	ACPI_HANDLE	handle;
	const struct nct_chip	*chip;
	struct asus_hwm_fan	fan[ASUS_HWM_MAXFAN];
};

static void	acpi_asus_hwm_identify(driver_t *, device_t);
static int	acpi_asus_hwm_probe(device_t);
static int	acpi_asus_hwm_attach(device_t);
static int	acpi_asus_hwm_detach(device_t);
static int	acpi_asus_hwm_suspend(device_t);
static int	acpi_asus_hwm_resume(device_t);
static int	acpi_asus_hwm_shutdown(device_t);
static int	acpi_asus_hwm_call(struct acpi_asus_hwm_softc *, uint32_t,
		    uint8_t, uint8_t, uint8_t, uint8_t *);
static int	acpi_asus_hwm_read(struct acpi_asus_hwm_softc *, uint16_t,
		    uint8_t *);
static int	acpi_asus_hwm_write(struct acpi_asus_hwm_softc *, uint16_t,
		    uint8_t);
static int	acpi_asus_hwm_read_word(struct acpi_asus_hwm_softc *,
		    uint16_t, uint16_t *);
static int	acpi_asus_hwm_save(struct acpi_asus_hwm_softc *);
static int	acpi_asus_hwm_acquire(struct acpi_asus_hwm_softc *, int);
static int	acpi_asus_hwm_release(struct acpi_asus_hwm_softc *, int);
static void	acpi_asus_hwm_restore(struct acpi_asus_hwm_softc *);
static int	acpi_asus_hwm_update(struct acpi_asus_hwm_softc *, int, int,
		    int *, bool);
static int	acpi_asus_hwm_sysctl_handler(SYSCTL_HANDLER_ARGS);
static int	acpi_asus_hwm_channel_count(device_t, u_int *);
static int	acpi_asus_hwm_channel_config(device_t, u_int, u_int, u_int);
static int	acpi_asus_hwm_channel_get_config(device_t, u_int, u_int *,
		    u_int *);
static int	acpi_asus_hwm_channel_set_flags(device_t, u_int, uint32_t);
static int	acpi_asus_hwm_channel_enable(device_t, u_int, bool);
static int	acpi_asus_hwm_channel_is_enabled(device_t, u_int, bool *);

/* per-fan controls */
enum {
	ASUS_HWM_CTRL_RPM	= 1,	/* Fan speed */
	ASUS_HWM_CTRL_PWM	= 2,	/* Fan duty cycle */
	ASUS_HWM_CTRL_MANUAL	= 3,	/* Manual control */
};
#define	ASUS_HWM_CTRL_MAX	4

struct asus_hwm_ctrl_table {
	char	*name;
	bool	rdonly;
	char	*desc;
};

static const struct asus_hwm_ctrl_table asus_hwm_sysctl_table[] = {
	[ASUS_HWM_CTRL_RPM] = {
		.name = "rpm",
		.rdonly = true,
		.desc = "Fan speed (RPM)",
	},
	[ASUS_HWM_CTRL_PWM] = {
		.name = "pwm",
		.desc = "Fan duty cycle (0-255), writable in manual mode",
	},
	[ASUS_HWM_CTRL_MANUAL] = {
		.name = "manual",
		.desc = "Manual fan control (0 = firmware curve, 1 = manual)",
	},
};

#define	ASUS_HWM_ARG(fan, ctrl)	((fan) << 8 | (ctrl))
#define	ASUS_HWM_ARG_FAN(arg)	((arg) >> 8)
#define	ASUS_HWM_ARG_CTRL(arg)	((arg) & 0xff)

static device_method_t acpi_asus_hwm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify, acpi_asus_hwm_identify),
	DEVMETHOD(device_probe, acpi_asus_hwm_probe),
	DEVMETHOD(device_attach, acpi_asus_hwm_attach),
	DEVMETHOD(device_detach, acpi_asus_hwm_detach),
	DEVMETHOD(device_suspend, acpi_asus_hwm_suspend),
	DEVMETHOD(device_resume, acpi_asus_hwm_resume),
	DEVMETHOD(device_shutdown, acpi_asus_hwm_shutdown),

	/* PWM interface */
	DEVMETHOD(pwmbus_channel_count, acpi_asus_hwm_channel_count),
	DEVMETHOD(pwmbus_channel_config, acpi_asus_hwm_channel_config),
	DEVMETHOD(pwmbus_channel_get_config, acpi_asus_hwm_channel_get_config),
	DEVMETHOD(pwmbus_channel_set_flags, acpi_asus_hwm_channel_set_flags),
	DEVMETHOD(pwmbus_channel_enable, acpi_asus_hwm_channel_enable),
	DEVMETHOD(pwmbus_channel_is_enabled, acpi_asus_hwm_channel_is_enabled),

	DEVMETHOD_END
};

static driver_t acpi_asus_hwm_driver = {
	"acpi_asus_hwm",
	acpi_asus_hwm_methods,
	sizeof(struct acpi_asus_hwm_softc)
};

/*
 * Evaluate WMBD(0, method, Buffer{bank, reg, val, 0}).
 * The firmware returns a byte on success and Ones on failure.
 */
static int
acpi_asus_hwm_call(struct acpi_asus_hwm_softc *sc, uint32_t method,
    uint8_t bank, uint8_t reg, uint8_t val, uint8_t *out)
{
	ACPI_STATUS status;
	ACPI_BUFFER Buf;
	ACPI_OBJECT Arg[3], Obj;
	ACPI_OBJECT_LIST Args;
	uint8_t data[4] = { bank, reg, val, 0 };

	ACPI_SERIAL_ASSERT(asus_hwm);

	Arg[0].Type = ACPI_TYPE_INTEGER;
	Arg[0].Integer.Value = 0;
	Arg[1].Type = ACPI_TYPE_INTEGER;
	Arg[1].Integer.Value = method;
	Arg[2].Type = ACPI_TYPE_BUFFER;
	Arg[2].Buffer.Length = sizeof(data);
	Arg[2].Buffer.Pointer = data;
	Args.Count = nitems(Arg);
	Args.Pointer = Arg;
	Buf.Length = sizeof(Obj);
	Buf.Pointer = &Obj;

	status = AcpiEvaluateObjectTyped(sc->handle, ASUS_HWM_METHOD, &Args,
	    &Buf, ACPI_TYPE_INTEGER);
	if (ACPI_FAILURE(status) || Obj.Integer.Value > UINT8_MAX)
		return (EIO);
	if (out != NULL)
		*out = Obj.Integer.Value;

	return (0);
}

static int
acpi_asus_hwm_read(struct acpi_asus_hwm_softc *sc, uint16_t reg,
    uint8_t *val)
{

	return (acpi_asus_hwm_call(sc, ASUS_HWM_METHODID_RHWM, reg >> 8,
	    reg & 0xff, 0, val));
}

static int
acpi_asus_hwm_write(struct acpi_asus_hwm_softc *sc, uint16_t reg,
    uint8_t val)
{

	return (acpi_asus_hwm_call(sc, ASUS_HWM_METHODID_WHWM, reg >> 8,
	    reg & 0xff, val, NULL));
}

/* Word registers hold the high byte at reg and the low byte at reg + 1. */
static int
acpi_asus_hwm_read_word(struct acpi_asus_hwm_softc *sc, uint16_t reg,
    uint16_t *val)
{
	uint8_t hi, lo;
	int error;

	if ((error = acpi_asus_hwm_read(sc, reg, &hi)) != 0 ||
	    (error = acpi_asus_hwm_read(sc, reg + 1, &lo)) != 0)
		return (error);
	*val = hi << 8 | lo;

	return (0);
}

/*
 * Snapshot the firmware fan configuration, so it can be put back when we
 * give up control.
 */
static int
acpi_asus_hwm_save(struct acpi_asus_hwm_softc *sc)
{
	struct asus_hwm_fan *fan;
	int i, error;

	ACPI_SERIAL_ASSERT(asus_hwm);

	for (i = 0; i < sc->chip->nfan; i++) {
		fan = &sc->fan[i];
		if ((error = acpi_asus_hwm_read(sc, nct_reg_fan_mode[i],
		    &fan->saved_mode)) != 0 ||
		    (error = acpi_asus_hwm_read(sc, nct_reg_pwm[i],
		    &fan->saved_pwm)) != 0)
			return (error);
		fan->manual = false;
		fan->duty_set = false;
	}

	return (0);
}

/*
 * Put a fan under manual control, starting from its current duty so it
 * doesn't jump.
 */
static int
acpi_asus_hwm_acquire(struct acpi_asus_hwm_softc *sc, int nr)
{
	struct asus_hwm_fan *fan;
	uint8_t mode, pwm;
	int error;

	ACPI_SERIAL_ASSERT(asus_hwm);

	fan = &sc->fan[nr];
	if (fan->manual)
		return (0);
	if ((error = acpi_asus_hwm_read(sc, nct_reg_pwm_read[nr], &pwm)) != 0 ||
	    (error = acpi_asus_hwm_read(sc, nct_reg_fan_mode[nr], &mode)) != 0)
		return (error);
	fan->manual = true;
	mode &= ~NCT_FAN_MODE_MASK;
	mode |= NCT_FAN_MODE_MANUAL << NCT_FAN_MODE_SHIFT;
	if ((error = acpi_asus_hwm_write(sc, nct_reg_pwm[nr], pwm)) != 0 ||
	    (error = acpi_asus_hwm_write(sc, nct_reg_fan_mode[nr], mode)) != 0)
		(void)acpi_asus_hwm_release(sc, nr);

	return (error);
}

/*
 * Hand a fan back to the firmware curve.
 */
static int
acpi_asus_hwm_release(struct acpi_asus_hwm_softc *sc, int nr)
{
	struct asus_hwm_fan *fan;
	int error;

	ACPI_SERIAL_ASSERT(asus_hwm);

	fan = &sc->fan[nr];
	if ((error = acpi_asus_hwm_write(sc, nct_reg_fan_mode[nr],
	    fan->saved_mode)) != 0 ||
	    (error = acpi_asus_hwm_write(sc, nct_reg_pwm[nr],
	    fan->saved_pwm)) != 0) {
		device_printf(sc->dev,
		    "Couldn't restore firmware control of fan%d\n", nr + 1);
		return (error);
	}
	fan->manual = false;

	return (0);
}

static void
acpi_asus_hwm_restore(struct acpi_asus_hwm_softc *sc)
{
	int i;

	ACPI_SERIAL_ASSERT(asus_hwm);

	for (i = 0; i < sc->chip->nfan; i++)
		if (sc->fan[i].manual)
			(void)acpi_asus_hwm_release(sc, i);
}

static int
acpi_asus_hwm_update(struct acpi_asus_hwm_softc *sc, int nr, int ctrl,
    int *val, bool set)
{
	struct asus_hwm_fan *fan;
	uint16_t rpm;
	uint8_t pwm;
	int error;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);
	ACPI_SERIAL_ASSERT(asus_hwm);

	fan = &sc->fan[nr];
	switch (ctrl) {
	case ASUS_HWM_CTRL_RPM:
		error = acpi_asus_hwm_read_word(sc, nct_reg_fan[nr], &rpm);
		if (error == 0)
			*val = rpm;
		break;
	case ASUS_HWM_CTRL_PWM:
		if (set) {
			if (!fan->manual)
				return (EBUSY);
			error = acpi_asus_hwm_write(sc, nct_reg_pwm[nr], *val);
		} else {
			/* The duty register is stale under firmware control. */
			error = acpi_asus_hwm_read(sc, fan->manual ?
			    nct_reg_pwm[nr] : nct_reg_pwm_read[nr], &pwm);
			if (error == 0)
				*val = pwm;
		}
		break;
	case ASUS_HWM_CTRL_MANUAL:
		if (!set) {
			*val = fan->manual;
			return (0);
		}
		if (*val == fan->manual)
			return (0);
		if (*val == 0)
			error = acpi_asus_hwm_release(sc, nr);
		else
			error = acpi_asus_hwm_acquire(sc, nr);
		break;
	default:
		return (EINVAL);
	}

	if (error != 0)
		device_printf(sc->dev, "Couldn't query fan%d %s\n", nr + 1,
		    asus_hwm_sysctl_table[ctrl].name);

	return (error);
}

static int
acpi_asus_hwm_sysctl_handler(SYSCTL_HANDLER_ARGS)
{
	struct acpi_asus_hwm_softc *sc;
	int val, nr, ctrl, error;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);

	sc = (struct acpi_asus_hwm_softc *)oidp->oid_arg1;
	nr = ASUS_HWM_ARG_FAN(oidp->oid_arg2);
	ctrl = ASUS_HWM_ARG_CTRL(oidp->oid_arg2);

	ACPI_SERIAL_BEGIN(asus_hwm);
	error = acpi_asus_hwm_update(sc, nr, ctrl, &val, false);
	ACPI_SERIAL_END(asus_hwm);
	if (error != 0)
		return (error);

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	/* Input validation */
	switch (ctrl) {
	case ASUS_HWM_CTRL_PWM:
		if (val > UINT8_MAX || val < 0)
			return (EINVAL);
		break;
	case ASUS_HWM_CTRL_MANUAL:
		if (val != 0 && val != 1)
			return (EINVAL);
		break;
	default:
		return (EINVAL);
	}

	ACPI_SERIAL_BEGIN(asus_hwm);
	error = acpi_asus_hwm_update(sc, nr, ctrl, &val, true);
	ACPI_SERIAL_END(asus_hwm);
	return (error);
}

static int
acpi_asus_hwm_channel_count(device_t dev, u_int *nchannel)
{
	struct acpi_asus_hwm_softc *sc;

	sc = device_get_softc(dev);
	*nchannel = sc->chip->nfan;

	return (0);
}

/*
 * The fan duty register has 8 bits, so the requested duty is scaled to
 * 0-255 relative to the requested period.  pwmc(4) consumers that leave
 * the period at 0 would otherwise turn every percentage into 0 and stop
 * the fan, so a period of 0 is rejected.
 */
static int
acpi_asus_hwm_channel_config(device_t dev, u_int channel, u_int period,
    u_int duty)
{
	struct acpi_asus_hwm_softc *sc;
	struct asus_hwm_fan *fan;
	uint8_t pwm;
	int error;

	sc = device_get_softc(dev);
	if (channel >= sc->chip->nfan || period == 0 || duty > period)
		return (EINVAL);

	pwm = ((uint64_t)duty * UINT8_MAX + period / 2) / period;
	fan = &sc->fan[channel];

	ACPI_SERIAL_BEGIN(asus_hwm);
	error = 0;
	if (fan->manual)
		error = acpi_asus_hwm_write(sc, nct_reg_pwm[channel], pwm);
	if (error == 0) {
		fan->period = period;
		fan->duty = pwm;
		fan->duty_set = true;
	}
	ACPI_SERIAL_END(asus_hwm);

	return (error);
}

static int
acpi_asus_hwm_channel_get_config(device_t dev, u_int channel, u_int *period,
    u_int *duty)
{
	struct acpi_asus_hwm_softc *sc;
	struct asus_hwm_fan *fan;
	uint8_t pwm;
	int error;

	sc = device_get_softc(dev);
	if (channel >= sc->chip->nfan)
		return (EINVAL);
	fan = &sc->fan[channel];

	ACPI_SERIAL_BEGIN(asus_hwm);
	error = acpi_asus_hwm_read(sc, fan->manual ? nct_reg_pwm[channel] :
	    nct_reg_pwm_read[channel], &pwm);
	if (error == 0) {
		*period = fan->period;
		*duty = ((uint64_t)pwm * fan->period + UINT8_MAX / 2) /
		    UINT8_MAX;
	}
	ACPI_SERIAL_END(asus_hwm);

	return (error);
}

static int
acpi_asus_hwm_channel_set_flags(device_t dev, u_int channel, uint32_t flags)
{
	struct acpi_asus_hwm_softc *sc;

	sc = device_get_softc(dev);
	if (channel >= sc->chip->nfan)
		return (EINVAL);

	/* Fan headers can't invert their output. */
	return ((flags & PWM_POLARITY_INVERTED) != 0 ? EINVAL : 0);
}

static int
acpi_asus_hwm_channel_enable(device_t dev, u_int channel, bool enable)
{
	struct acpi_asus_hwm_softc *sc;
	struct asus_hwm_fan *fan;
	int error;

	sc = device_get_softc(dev);
	if (channel >= sc->chip->nfan)
		return (EINVAL);
	fan = &sc->fan[channel];

	ACPI_SERIAL_BEGIN(asus_hwm);
	if (!enable)
		error = fan->manual ? acpi_asus_hwm_release(sc, channel) : 0;
	else if ((error = acpi_asus_hwm_acquire(sc, channel)) == 0 &&
	    fan->duty_set)
		error = acpi_asus_hwm_write(sc, nct_reg_pwm[channel],
		    fan->duty);
	ACPI_SERIAL_END(asus_hwm);

	return (error);
}

static int
acpi_asus_hwm_channel_is_enabled(device_t dev, u_int channel, bool *enabled)
{
	struct acpi_asus_hwm_softc *sc;

	sc = device_get_softc(dev);
	if (channel >= sc->chip->nfan)
		return (EINVAL);
	*enabled = sc->fan[channel].manual;

	return (0);
}

static void
acpi_asus_hwm_identify(driver_t *driver, device_t parent)
{
	ACPI_DEVICE_INFO *info;
	ACPI_HANDLE handle, method;
	bool match;

	if (acpi_disabled("asus_hwm"))
		return;

	if (device_find_child(parent, "acpi_asus_hwm", DEVICE_UNIT_ANY) != NULL)
		return;

	if ((handle = acpi_get_handle(parent)) == NULL ||
	    ACPI_FAILURE(AcpiGetHandle(handle, ASUS_HWM_METHOD, &method)))
		return;

	if (ACPI_FAILURE(AcpiGetObjectInfo(handle, &info)))
		return;
	match = (info->Valid & ACPI_VALID_UID) != 0 &&
	    (strcmp(info->UniqueId.String, ASUS_HWM_UID_MSI) == 0 ||
	    strcmp(info->UniqueId.String, ASUS_HWM_UID_WMI) == 0);
	AcpiOsFree(info);
	if (!match)
		return;

	if (BUS_ADD_CHILD(parent, 0, "acpi_asus_hwm", DEVICE_UNIT_ANY) == NULL)
		device_printf(parent, "add acpi_asus_hwm child failed\n");
}

static int
acpi_asus_hwm_probe(device_t dev)
{

	if (acpi_disabled("asus_hwm"))
		return (ENXIO);
	device_set_desc(dev, "ASUS hardware monitor");

	return (BUS_PROBE_VENDOR);
}

static int
acpi_asus_hwm_attach(device_t dev)
{
	struct acpi_asus_hwm_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree, *node;
	const struct nct_chip *chip;
	char name[8];
	uint16_t id;
	uint8_t hi, lo, chipid;
	int i, ctrl, error;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->handle = acpi_get_handle(device_get_parent(dev));

	ACPI_SERIAL_BEGIN(asus_hwm);
	error = acpi_asus_hwm_call(sc, ASUS_HWM_METHODID_RSIO, NCT_LD_HWM,
	    NCT_SIO_REG_DEVID, 0, &hi);
	if (error == 0)
		error = acpi_asus_hwm_call(sc, ASUS_HWM_METHODID_RSIO,
		    NCT_LD_HWM, NCT_SIO_REG_DEVID + 1, 0, &lo);
	if (error == 0)
		error = acpi_asus_hwm_read(sc, NCT_REG_CHIPID, &chipid);
	if (error != 0) {
		ACPI_SERIAL_END(asus_hwm);
		device_printf(dev, "Couldn't query the Super I/O chip\n");
		return (ENXIO);
	}

	id = hi << 8 | lo;
	for (chip = nct_chips; chip->name != NULL; chip++)
		if ((id & NCT_SIO_ID_MASK) == chip->id)
			break;
	if (chip->name == NULL || chipid != NCT_CHIPID) {
		ACPI_SERIAL_END(asus_hwm);
		device_printf(dev,
		    "Unsupported chip (id %#06x, chip id %#04x)\n", id, chipid);
		return (ENXIO);
	}
	sc->chip = chip;
	for (i = 0; i < chip->nfan; i++)
		sc->fan[i].period = ASUS_HWM_PWM_PERIOD;

	if (acpi_asus_hwm_save(sc) != 0) {
		ACPI_SERIAL_END(asus_hwm);
		device_printf(dev, "Couldn't read the fan configuration\n");
		return (ENXIO);
	}
	ACPI_SERIAL_END(asus_hwm);

	device_printf(dev, "Nuvoton %s (id %#06x), %d fans\n", chip->name, id,
	    chip->nfan);

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	for (i = 0; i < chip->nfan; i++) {
		snprintf(name, sizeof(name), "fan%d", i + 1);
		node = SYSCTL_ADD_NODE(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    name, CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "Fan");

		for (ctrl = 1; ctrl < ASUS_HWM_CTRL_MAX; ctrl++) {
			if (asus_hwm_sysctl_table[ctrl].rdonly)
				SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(node),
				    OID_AUTO, asus_hwm_sysctl_table[ctrl].name,
				    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
				    sc, ASUS_HWM_ARG(i, ctrl),
				    acpi_asus_hwm_sysctl_handler, "I",
				    asus_hwm_sysctl_table[ctrl].desc);
			else
				SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(node),
				    OID_AUTO, asus_hwm_sysctl_table[ctrl].name,
				    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
				    sc, ASUS_HWM_ARG(i, ctrl),
				    acpi_asus_hwm_sysctl_handler, "I",
				    asus_hwm_sysctl_table[ctrl].desc);
		}
	}

	sc->busdev = device_add_child(dev, "pwmbus", DEVICE_UNIT_ANY);
	if (sc->busdev == NULL)
		device_printf(dev, "Couldn't add pwmbus child\n");
	bus_attach_children(dev);

	return (0);
}

static int
acpi_asus_hwm_detach(device_t dev)
{
	int error;

	if ((error = bus_generic_detach(dev)) != 0)
		return (error);

	return (acpi_asus_hwm_shutdown(dev));
}

static int
acpi_asus_hwm_suspend(device_t dev)
{

	return (acpi_asus_hwm_shutdown(dev));
}

/*
 * The firmware reprograms the chip on resume, so take a fresh snapshot
 * instead of reapplying manual settings.
 */
static int
acpi_asus_hwm_resume(device_t dev)
{
	struct acpi_asus_hwm_softc *sc;

	sc = device_get_softc(dev);

	ACPI_SERIAL_BEGIN(asus_hwm);
	if (acpi_asus_hwm_save(sc) != 0)
		device_printf(dev, "Couldn't read the fan configuration\n");
	ACPI_SERIAL_END(asus_hwm);

	return (0);
}

/*
 * Hand every fan back to the firmware curve.
 */
static int
acpi_asus_hwm_shutdown(device_t dev)
{
	struct acpi_asus_hwm_softc *sc;

	sc = device_get_softc(dev);

	ACPI_SERIAL_BEGIN(asus_hwm);
	acpi_asus_hwm_restore(sc);
	ACPI_SERIAL_END(asus_hwm);

	return (0);
}

DRIVER_MODULE(acpi_asus_hwm, acpi_wmi, acpi_asus_hwm_driver, 0, 0);
DRIVER_MODULE(pwmbus, acpi_asus_hwm, pwmbus_driver, 0, 0);
MODULE_VERSION(acpi_asus_hwm, 1);
MODULE_DEPEND(acpi_asus_hwm, acpi_wmi, 1, 1, 1);
MODULE_DEPEND(acpi_asus_hwm, acpi, 1, 1, 1);
MODULE_DEPEND(acpi_asus_hwm, pwmbus, 1, 1, 1);
MODULE_DEPEND(acpi_asus_hwm, pwmc, 1, 1, 1);
