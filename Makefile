# Smart Car — Makefile
# Target: Orange Pi 5B (aarch64), Ubuntu 22.04/24.04
# Author: Ltttttts

CC       := gcc
CFLAGS   := -Wall -Wextra -Wconversion -Wshadow \
            -std=c11 -O2 -g
LDFLAGS  := -lm -lpthread

SRCDIR   := src
BINDIR   := build

INC      := -I$(SRCDIR)

# 共享的驱动模块（不含 main.c 和 teleop.c）
DRV_SRCS  := $(SRCDIR)/serial_port.c \
             $(SRCDIR)/emm_v5_driver.c \
             $(SRCDIR)/kinematics.c
DRV_OBJS  := $(patsubst $(SRCDIR)/%.c, $(BINDIR)/%.o, $(DRV_SRCS))

# 两个可执行程序
TARGET_DEMO   := $(BINDIR)/smart_car
TARGET_TELEOP := $(BINDIR)/teleop

.PHONY: all clean run run-teleop dist help

all: $(TARGET_DEMO) $(TARGET_TELEOP)

$(BINDIR):
	mkdir -p $(BINDIR)

# ---- 编译规则 ----

$(BINDIR)/%.o: $(SRCDIR)/%.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INC) -MMD -MP -c $< -o $@
	@echo "  CC    $<"

# ---- 链接 ----

$(TARGET_DEMO): $(BINDIR)/main.o $(DRV_OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	@echo "  LINK  $@"

$(TARGET_TELEOP): $(BINDIR)/teleop.o $(DRV_OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	@echo "  LINK  $@"

# 自动依赖文件
-include $(wildcard $(BINDIR)/*.d)

# ---- 运行 ----

run: $(TARGET_DEMO)
	@echo "=== Demo 演示 ==="
	./$(TARGET_DEMO) /dev/ttyUSB0

run-teleop: $(TARGET_TELEOP)
	@echo "=== 键盘遥控 ==="
	./$(TARGET_TELEOP) /dev/ttyUSB0

# ---- 清理 ----

clean:
	rm -rf $(BINDIR)

# ---- 打包 ----

dist:
	tar czf smart_car_src.tar.gz \
	    --exclude='build' \
	    --exclude='node_modules' \
	    --exclude='.git' \
	    --exclude='*.tar.gz' \
	    src/ Makefile PROPOSAL.md "reference data" .gitignore
	@echo "  DIST  smart_car_src.tar.gz"

help:
	@echo "Usage:"
	@echo "  make            编译全部"
	@echo "  make run        运行 Demo 演示"
	@echo "  make run-teleop 运行键盘遥控"
	@echo "  make clean      清理编译产物"
	@echo "  make dist       打包源码"
	@echo ""
	@echo "  硬件开关: 修改 src/main.c 或 src/teleop.c 顶部"
	@echo "            HARDWARE_ENABLED 0→1 后重新 make"
