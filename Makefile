# Smart Car — Makefile
# Target: Orange Pi 5B (aarch64), Ubuntu 22.04/24.04
# Author: Ltttttts

CC       := gcc
CFLAGS   := -Wall -Wextra -Wconversion -Wshadow \
            -std=c11 -O2 -g
LDFLAGS  := -lm -lpthread -lcurl

BINDIR   := build
INC      := -Iinclude -Imodules

# ── vpath：告诉 make 到哪些子目录找 .c 文件 ──
vpath %.c modules/hal:modules/driver:modules/control:modules/comm:modules/ui:app/joystick:app/ai

# ── 模块目标文件 ──
MOD_OBJS := $(BINDIR)/serial_port.o \
            $(BINDIR)/emm_v5.o \
            $(BINDIR)/kinematics.o \
            $(BINDIR)/llm_client.o \
            $(BINDIR)/json_helper.o \
            $(BINDIR)/dashboard.o

# ── 可执行程序 ──
TARGET_JOYSTICK := $(BINDIR)/joystick
TARGET_AI       := $(BINDIR)/ai_control

.PHONY: all clean run-joystick calibrate-joystick run-ai dist help

all: $(TARGET_JOYSTICK) $(TARGET_AI)

$(BINDIR):
	mkdir -p $(BINDIR)

# ── 通用编译规则（vpath 自动找源文件） ──
$(BINDIR)/%.o: %.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INC) -MMD -MP -c $< -o $@
	@echo "  CC    $<"

# ── 链接 ──
$(TARGET_JOYSTICK): $(BINDIR)/main.o $(MOD_OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	@echo "  LINK  joystick"

$(TARGET_AI): $(BINDIR)/ai_control.o $(MOD_OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	@echo "  LINK  ai_control"

# 自动依赖文件
-include $(wildcard $(BINDIR)/*.d)

# ── 运行 ──
run-joystick: $(TARGET_JOYSTICK)
	@echo "=== 手柄遥控 ==="
	./$(TARGET_JOYSTICK) /dev/ttyUSB0

calibrate-joystick: $(TARGET_JOYSTICK)
	@echo "=== 手柄校准 ==="
	./$(TARGET_JOYSTICK) /dev/ttyUSB0 --calibrate

run-ai: $(TARGET_AI)
	@echo "=== AI 控制 ==="
	./$(TARGET_AI) /dev/ttyUSB0

# ── 清理 ──
clean:
	rm -rf $(BINDIR)

# ── 打包 ──
dist:
	tar czf smart_car_src.tar.gz \
	    --exclude='build' --exclude='node_modules' \
	    --exclude='.git' --exclude='*.tar.gz' \
	    include/ modules/ app/ config/ docs/ scripts/ tests/ \
	    Makefile PROPOSAL.md .gitignore
	@echo "  DIST  smart_car_src.tar.gz"

help:
	@echo "Usage:"
	@echo "  make                     编译全部"
	@echo "  make run-joystick        运行手柄遥控"
	@echo "  make calibrate-joystick  手柄校准"
	@echo "  make run-ai              AI 对话控制"
	@echo "  make clean               清理编译产物"
	@echo "  make dist                打包源码"
	@echo ""
	@echo "  AI 模式需要设置环境变量:"
	@echo "    export LLM_API_KEY=sk-xxxxx"
	@echo " 硬件开关: 修改 app/*/main.c 顶部"
	@echo "            HARDWARE_ENABLED 0->1 后重新 make"
