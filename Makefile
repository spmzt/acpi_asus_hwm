KMOD=	acpi_asus_hwm
SRCS=	acpi_asus_hwm.c
SRCS+=	opt_acpi.h acpi_if.h bus_if.h device_if.h pwmbus_if.h

.include <bsd.kmod.mk>
