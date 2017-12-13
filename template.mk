LIBNAME=okapilib
VERSION=0.5.1

# extra files (like header files)
TEMPLATEFILES = include/main.h include/device/motor.h include/device/button.h include/device/ime.h include/device/potentiometer.h include/device/quadEncoder.h include/device/rangeFinder.h include/device/rotarySensor.h include/chassis/chassisModel.h include/chassis/odomChassisController.h include/chassis/chassisController.h include/API.h include/util/timer.h include/util/mathUtil.h include/odometry/odomMath.h include/odometry/odometry.h include/filter/filter.h include/filter/emaFilter.h include/filter/avgFilter.h include/filter/demaFilter.h include/control/pid.h include/control/genericController.h include/control/velMath.h include/control/nsPid.h include/control/velPid.h include/control/controlObject.h
# basename of the source files that should be archived
TEMPLATEOBJS = _bin_auto _bin_chassis_chassisController _bin_chassis_odomChassisController _bin_control_nsPid _bin_control_pid _bin_control_velPid _bin_init _bin_odometry_odometry _bin_odometry_odomMath _bin_opcontrol _bin_util_timer

TEMPLATE=$(ROOT)/$(LIBNAME)-template

.DEFAULT_GOAL: all

library: clean $(BINDIR) $(SUBDIRS) $(ASMOBJ) $(COBJ) $(CPPOBJ)
	$(MCUPREFIX)ar rvs $(BINDIR)/$(LIBNAME).a $(foreach f,$(TEMPLATEOBJS),$(BINDIR)/$(f).o)
	mkdir -p $(TEMPLATE) $(TEMPLATE)/firmware $(addprefix $(TEMPLATE)/, $(dir $(TEMPLATEFILES)))
	cp $(BINDIR)/$(LIBNAME).a $(TEMPLATE)/firmware/$(LIBNAME).a
	$(foreach f,$(TEMPLATEFILES),cp $(f) $(TEMPLATE)/$(f);)
	pros conduct create-template $(LIBNAME) $(VERSION) $(TEMPLATE) --ignore template.pros --upgrade-files firmware/$(LIBNAME).a $(foreach f,$(TEMPLATEFILES),--upgrade-files $(f))
	@echo Need to zip $(TEMPLATE) without the base directory
	cd $(TEMPLATE) && zip -r ../$(basename $(TEMPLATE)) *
