/**
 * @file    app/camera/main.c
 * @brief   摄像头 MJPG-Streamer 推流启动程序
 * @author  Ltttttts
 *
 * 功能：
 *   1. 根据命令行参数生成 mjpg_streamer 启动参数
 *   2. 切换到 mjpg_streamer 目录
 *   3. 用 execvp() 启动推流进程
 *
 * 使用方式：
 *   ./camera_stream [选项]
 *
 * 常用选项：
 *   --dir      mjpg_streamer 所在目录
 *   --device   摄像头设备，例如 /dev/video0
 *   --res      分辨率，例如 640x480
 *   --fps      帧率
 *   --quality  JPEG 质量
 *   --port     HTTP 端口
 *   --www      Web 静态文件目录
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DEFAULT_STREAMER_DIR \
    "/home/orangepi/Desktop/mjpg/mjpg-streamer/mjpg-streamer-experimental"
#define DEFAULT_DEVICE       "/dev/video0"
#define DEFAULT_RESOLUTION   "640x480"
#define DEFAULT_FPS          (30)
#define DEFAULT_QUALITY      (60)
#define DEFAULT_PORT         (8080)
#define DEFAULT_WWW_DIR      "./www"

#define PLUGIN_ARG_SIZE      (256U)

typedef struct {
    const char *streamer_dir;
    const char *device;
    const char *resolution;
    const char *www_dir;
    int         fps;
    int         quality;
    int         port;
} CameraStreamConfig_t;

static void print_usage(const char *prog)
{
    printf("用法: %s [选项]\n", prog);
    printf("选项:\n");
    printf("  --dir <路径>       mjpg_streamer 目录\n");
    printf("  --device <设备>    摄像头设备，默认 %s\n", DEFAULT_DEVICE);
    printf("  --res <宽x高>      分辨率，默认 %s\n", DEFAULT_RESOLUTION);
    printf("  --fps <帧率>       默认 %d\n", DEFAULT_FPS);
    printf("  --quality <质量>   JPEG 质量，默认 %d\n", DEFAULT_QUALITY);
    printf("  --port <端口>      HTTP 端口，默认 %d\n", DEFAULT_PORT);
    printf("  --www <路径>       Web 目录，默认 %s\n", DEFAULT_WWW_DIR);
    printf("  --help             显示帮助\n");
}

static int parse_int_arg(const char *text, int min, int max, int *out)
{
    char *end = NULL;
    long value;

    if (text == NULL || out == NULL) {
        return -1;
    }

    errno = 0;
    value = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0') {
        return -1;
    }
    if (value < (long)min || value > (long)max) {
        return -1;
    }

    *out = (int)value;
    return 0;
}

static int require_value(int argc, char *argv[], int index)
{
    if ((index + 1) >= argc) {
        fprintf(stderr, "[错误] %s 需要一个参数\n", argv[index]);
        return -1;
    }
    return 0;
}

static int parse_args(int argc, char *argv[], CameraStreamConfig_t *cfg)
{
    int i;

    if (cfg == NULL) {
        return -1;
    }

    cfg->streamer_dir = DEFAULT_STREAMER_DIR;
    cfg->device       = DEFAULT_DEVICE;
    cfg->resolution   = DEFAULT_RESOLUTION;
    cfg->www_dir      = DEFAULT_WWW_DIR;
    cfg->fps          = DEFAULT_FPS;
    cfg->quality      = DEFAULT_QUALITY;
    cfg->port         = DEFAULT_PORT;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            exit(EXIT_SUCCESS);
        } else if (strcmp(argv[i], "--dir") == 0) {
            if (require_value(argc, argv, i) != 0) return -1;
            cfg->streamer_dir = argv[++i];
        } else if (strcmp(argv[i], "--device") == 0) {
            if (require_value(argc, argv, i) != 0) return -1;
            cfg->device = argv[++i];
        } else if (strcmp(argv[i], "--res") == 0) {
            if (require_value(argc, argv, i) != 0) return -1;
            cfg->resolution = argv[++i];
        } else if (strcmp(argv[i], "--www") == 0) {
            if (require_value(argc, argv, i) != 0) return -1;
            cfg->www_dir = argv[++i];
        } else if (strcmp(argv[i], "--fps") == 0) {
            if (require_value(argc, argv, i) != 0) return -1;
            if (parse_int_arg(argv[++i], 1, 120, &cfg->fps) != 0) {
                fprintf(stderr, "[错误] fps 必须在 1~120 之间\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--quality") == 0) {
            if (require_value(argc, argv, i) != 0) return -1;
            if (parse_int_arg(argv[++i], 1, 100, &cfg->quality) != 0) {
                fprintf(stderr, "[错误] quality 必须在 1~100 之间\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--port") == 0) {
            if (require_value(argc, argv, i) != 0) return -1;
            if (parse_int_arg(argv[++i], 1, 65535, &cfg->port) != 0) {
                fprintf(stderr, "[错误] port 必须在 1~65535 之间\n");
                return -1;
            }
        } else {
            fprintf(stderr, "[错误] 未知选项: %s\n", argv[i]);
            return -1;
        }
    }

    return 0;
}

static int build_plugin_args(const CameraStreamConfig_t *cfg,
                             char *input_arg,
                             size_t input_size,
                             char *output_arg,
                             size_t output_size)
{
    int ret;

    if (cfg == NULL || input_arg == NULL || output_arg == NULL) {
        return -1;
    }

    ret = snprintf(input_arg, input_size,
                   "./input_uvc.so -d %s -r %s -f %d -q %d",
                   cfg->device, cfg->resolution, cfg->fps, cfg->quality);
    if (ret < 0 || (size_t)ret >= input_size) {
        fprintf(stderr, "[错误] 输入插件参数过长\n");
        return -1;
    }

    ret = snprintf(output_arg, output_size,
                   "./output_http.so -w %s -p %d",
                   cfg->www_dir, cfg->port);
    if (ret < 0 || (size_t)ret >= output_size) {
        fprintf(stderr, "[错误] 输出插件参数过长\n");
        return -1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    CameraStreamConfig_t cfg;
    char input_arg[PLUGIN_ARG_SIZE];
    char output_arg[PLUGIN_ARG_SIZE];
    char *exec_args[] = {
        "./mjpg_streamer",
        "-i",
        input_arg,
        "-o",
        output_arg,
        NULL
    };

    if (parse_args(argc, argv, &cfg) != 0) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    if (build_plugin_args(&cfg, input_arg, sizeof(input_arg),
                          output_arg, sizeof(output_arg)) != 0) {
        return EXIT_FAILURE;
    }

    if (chdir(cfg.streamer_dir) != 0) {
        perror("[错误] 无法进入 mjpg_streamer 目录");
        return EXIT_FAILURE;
    }

    printf("[摄像头] 启动 MJPG-Streamer\n");
    printf("[摄像头] 目录: %s\n", cfg.streamer_dir);
    printf("[摄像头] 输入: %s\n", input_arg);
    printf("[摄像头] 输出: %s\n", output_arg);
    printf("[摄像头] 地址: http://<OrangePi-IP>:%d/\n", cfg.port);
    fflush(stdout);

    execvp(exec_args[0], exec_args);
    perror("[错误] 启动 mjpg_streamer 失败");
    return EXIT_FAILURE;
}
