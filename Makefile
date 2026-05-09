ARTIFACT = hmds
PLATFORM ?= x86_64
BUILD_PROFILE ?= debug
CONFIG_NAME ?= $(PLATFORM)-$(BUILD_PROFILE)
OUTPUT_DIR = build/$(CONFIG_NAME)
TARGET = $(OUTPUT_DIR)/$(ARTIFACT)
RADAR_SIM = $(OUTPUT_DIR)/radar_sim

CC = qcc -Vgcc_nto$(PLATFORM)
LD = $(CC)

INCLUDES += -Iinclude
LIBS += -lm

CCFLAGS_release += -O2
CCFLAGS_debug += -g -O0 -fno-builtin
CCFLAGS_all += -Wall -fmessage-length=0
CCFLAGS_all += $(CCFLAGS_$(BUILD_PROFILE))
LDFLAGS_all += $(LDFLAGS_$(BUILD_PROFILE))
LIBS_all += $(LIBS_$(BUILD_PROFILE))
DEPS = -Wp,-MMD,$(@:%.o=%.d),-MT,$@

# Main binary sources (includes hmds_utils.c and qnet_utils.c)
MAIN_SRCS = src/main.c \
            src/detection_task.c \
            src/tracking_task.c \
            src/intercept_calc.c \
            src/launch_command.c \
            src/watchdog.c \
            src/kalman.c \
            src/qnx_concepts.c \
            src/hmds_utils.c \
            src/qnet_utils.c

RADAR_SRCS = sim/radar_sim.c

MAIN_OBJS = $(addprefix $(OUTPUT_DIR)/,$(addsuffix .o, $(basename $(MAIN_SRCS))))
RADAR_OBJS = $(addprefix $(OUTPUT_DIR)/,$(addsuffix .o, $(basename $(RADAR_SRCS))))

$(OUTPUT_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(DEPS) -o $@ $(INCLUDES) $(CCFLAGS_all) $(CCFLAGS) $<

$(TARGET): $(MAIN_OBJS)
	$(LD) -o $(TARGET) $(LDFLAGS_all) $(LDFLAGS) $(MAIN_OBJS) $(LIBS_all) $(LIBS)

$(RADAR_SIM): $(RADAR_OBJS)
	$(LD) -o $(RADAR_SIM) $(LDFLAGS_all) $(LDFLAGS) $(RADAR_OBJS) $(LIBS_all) $(LIBS)

all: $(TARGET) $(RADAR_SIM)

clean:
	rm -fr $(OUTPUT_DIR)

rebuild: clean all

-include $(MAIN_OBJS:%.o=%.d)
-include $(RADAR_OBJS:%.o=%.d)
