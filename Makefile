# Smart Car — 顶层构建文件
# 目标平台: Orange Pi 5B (aarch64), Ubuntu 22.04/24.04
# 作者: Ltttttts
#
# 用法:
#   make               编译三个可执行文件
#   make run-joystick  运行手柄遥控模式
#   make run-ai        运行 AI 对话控制模式
#   make run-camera    运行摄像头推流模式
#   make clean         清理编译产物
#
# 编译产物:
#   build/joystick    手柄遥控程序
#   build/ai_control  AI 控制程序
#   build/camera_stream 摄像头推流启动程序

CC       := gcc

# 编译选项
#   -std=c11      使用 C11 标准
#   -O2           优化级别 2
#   -g            包含调试符号
#   -Wall -Wextra -Wconversion -Wshadow  打开所有常用警告
CFLAGS   := -Wall -Wextra -Wconversion -Wshadow \
            -std=c11 -O2 -g

# 链接库：数学库、线程库、libcurl（用于 LLM API 调用）
LDFLAGS  := -lm -lpthread -lcurl

BINDIR   := build
INC      := -Iinclude -Imodules

# 告诉 make 到这些子目录去找 .c 源文件
vpath %.c modules/hal:modules/driver:modules/control:modules/comm

# 所有模块的目标文件列表
MOD_OBJS := $(BINDIR)/serial_port.o \
            $(BINDIR)/emm_v5.o \
            $(BINDIR)/kinematics.o \
            $(BINDIR)/llm_client.o \
            $(BINDIR)/json_helper.o

# 三个可执行程序
TARGET_JOYSTICK := $(BINDIR)/joystick      # 手柄遥控
TARGET_AI       := $(BINDIR)/ai_control    # AI 对话控制
TARGET_CAMERA   := $(BINDIR)/camera_stream # 摄像头推流

.PHONY: all clean run-joystick calibrate-joystick run-ai run-camera dist help

all: $(TARGET_JOYSTICK) $(TARGET_AI) $(TARGET_CAMERA)

$(BINDIR):
	mkdir -p $(BINDIR)

# 通用编译规则：.c → .o（自动生成 .d 依赖文件）
$(BINDIR)/%.o: %.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INC) -MMD -MP -c $< -o $@
	@echo "  CC    $<"

$(BINDIR)/joystick_main.o: app/joystick/main.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INC) -MMD -MP -c $< -o $@
	@echo "  CC    $<"

$(BINDIR)/ai_control.o: app/ai/main.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INC) -MMD -MP -c $< -o $@
	@echo "  CC    $<"

$(BINDIR)/camera_stream.o: app/camera/main.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INC) -MMD -MP -c $< -o $@
	@echo "  CC    $<"

# 链接手柄遥控程序（joystick_main.o + 所有模块）
$(TARGET_JOYSTICK): $(BINDIR)/joystick_main.o $(MOD_OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	@echo "  LINK  joystick"

# 链接 AI 控制程序（ai_control.o + 所有模块）
$(TARGET_AI): $(BINDIR)/ai_control.o $(MOD_OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	@echo "  LINK  ai_control"

# 链接摄像头推流启动程序
$(TARGET_CAMERA): $(BINDIR)/camera_stream.o | $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $@
	@echo "  LINK  camera_stream"

# 自动包含 .d 依赖文件（跟踪头文件变化）
-include $(wildcard $(BINDIR)/*.d)

# ---- 运行目标 ----
run-joystick: $(TARGET_JOYSTICK)
	@echo "=== 手柄遥控 ==="
	./$(TARGET_JOYSTICK) /dev/ttyUSB0 $(JOYSTICK_ARGS)

calibrate-joystick: $(TARGET_JOYSTICK)
	@echo "=== 手柄校准 ==="
	./$(TARGET_JOYSTICK) /dev/ttyUSB0 --calibrate

run-ai: $(TARGET_AI)
	@echo "=== AI 控制 ==="
	./$(TARGET_AI) /dev/ttyUSB0

run-camera: $(TARGET_CAMERA)
	@echo "=== 摄像头推流 ==="
	./$(TARGET_CAMERA) $(CAMERA_ARGS)

# ---- 清理 ----
clean:
	rm -rf $(BINDIR)

# ---- 打包源码 ----
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
	@echo "  make run-camera          摄像头推流"
	@echo "  make clean               清理编译产物"
	@echo "  make dist                打包源码"
	@echo ""
	@echo "  摄像头参数示例:"
	@echo "    make run-camera CAMERA_ARGS='--device /dev/video0 --res 640x480 --fps 30 --quality 60 --port 8080'"
	@echo "    make run-joystick JOYSTICK_ARGS='--camera --camera-device /dev/video0 --camera-res 640x480 --camera-fps 30 --camera-quality 60 --camera-port 8080'"
	@echo ""
	@echo "  AI 模式需要设置环境变量:"
	@echo "    export LLM_API_KEY=sk-xxxxx"
	@echo " 硬件开关: 修改 app/*/main.c 顶部"
	@echo "            HARDWARE_ENABLED 0->1 后重新 make"
