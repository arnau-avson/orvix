/*
 * lens.c - native screenshot streamer with tap-overlay support
 *
 * Wrapper minimo en C alrededor de /system/bin/screencap. Modo `tap-stream`
 * combina:
 *   - capturas periodicas cada N ms (POST /upload)
 *   - capturas inmediatas en cada tap (POST /upload?x=...&y=...)
 *
 * Las coords del tap salen de /system/bin/getevent -l, parseando los eventos
 * EV_ABS ABS_MT_POSITION_X/Y y EV_KEY BTN_TOUCH del touchscreen.
 *
 * Build (NDK clang, x86_64 Android):
 *   x86_64-linux-android30-clang -O3 -o lens lens.c
 *
 * Use:
 *   lens shot       URL                   una captura
 *   lens stream     URL [interval_ms]     periodicas (sin taps)
 *   lens tap-stream URL [interval_ms]     periodicas + por tap
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>

typedef struct {
    char host[256];
    int  port;
    char path[512];
} url_t;

typedef struct {
    int x, y;
    int has_x, has_y;
    int touch_down;
} tap_state_t;

/* ---------- url + http helpers ---------- */

static int parse_url(const char *s, url_t *u) {
    if (strncmp(s, "http://", 7) != 0) return -1;
    s += 7;
    const char *colon = strchr(s, ':');
    const char *slash = strchr(s, '/');
    const char *hostend;
    if (colon && (!slash || colon < slash)) hostend = colon;
    else if (slash)                          hostend = slash;
    else                                     hostend = s + strlen(s);
    size_t hlen = (size_t)(hostend - s);
    if (hlen == 0 || hlen >= sizeof u->host) return -1;
    memcpy(u->host, s, hlen); u->host[hlen] = 0;
    u->port = 80;
    if (colon && (!slash || colon < slash)) u->port = atoi(colon + 1);
    if (u->port <= 0 || u->port > 65535) return -1;
    if (slash) {
        if (strlen(slash) >= sizeof u->path) return -1;
        strcpy(u->path, slash);
    } else {
        strcpy(u->path, "/");
    }
    return 0;
}

static int tcp_connect(const char *host, int port) {
    char ports[16]; snprintf(ports, sizeof ports, "%d", port);
    struct addrinfo hints = {0}, *res = NULL;
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, ports, &hints, &res) != 0) return -1;
    int fd = -1;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) break;
        close(fd); fd = -1;
    }
    freeaddrinfo(res);
    if (fd >= 0) {
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    }
    return fd;
}

static int send_all(int fd, const void *buf, size_t n) {
    const char *p = (const char *)buf;
    while (n) {
        ssize_t w = send(fd, p, n, MSG_NOSIGNAL);
        if (w < 0) { if (errno == EINTR) continue; return -1; }
        p += w; n -= (size_t)w;
    }
    return 0;
}

/* ---------- screencap ---------- */

static char *capture_png(size_t *out_len) {
    int p[2];
    if (pipe(p) < 0) return NULL;
    pid_t pid = fork();
    if (pid < 0) { close(p[0]); close(p[1]); return NULL; }
    if (pid == 0) {
        close(p[0]);
        dup2(p[1], STDOUT_FILENO);
        int dn = open("/dev/null", O_WRONLY | O_CLOEXEC);
        if (dn >= 0) { dup2(dn, STDERR_FILENO); close(dn); }
        close(p[1]);
        execl("/system/bin/screencap", "screencap", "-p", (char *)NULL);
        _exit(127);
    }
    close(p[1]);
    size_t cap = 1u << 18, len = 0;
    char *buf = (char *)malloc(cap);
    if (!buf) { close(p[0]); waitpid(pid, NULL, 0); return NULL; }
    for (;;) {
        if (len == cap) {
            size_t nc = cap * 2;
            char *nb = (char *)realloc(buf, nc);
            if (!nb) { free(buf); close(p[0]); waitpid(pid, NULL, 0); return NULL; }
            buf = nb; cap = nc;
        }
        ssize_t n = read(p[0], buf + len, cap - len);
        if (n < 0) { if (errno == EINTR) continue; free(buf); close(p[0]); waitpid(pid, NULL, 0); return NULL; }
        if (n == 0) break;
        len += (size_t)n;
    }
    close(p[0]);
    int status = 0;
    waitpid(pid, &status, 0);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) { free(buf); return NULL; }
    *out_len = len;
    return buf;
}

static int post_png(const url_t *u, const char *path, const char *png, size_t len) {
    int sock = tcp_connect(u->host, u->port);
    if (sock < 0) { perror("connect"); return -1; }
    char hdr[1024];
    int hl = snprintf(hdr, sizeof hdr,
        "POST %s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "User-Agent: lens/1.0\r\n"
        "Content-Type: image/png\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n"
        "\r\n",
        path, u->host, u->port, len);
    if (hl <= 0 || (size_t)hl >= sizeof hdr) { close(sock); return -1; }
    int rc = -1;
    if (send_all(sock, hdr, (size_t)hl) == 0 &&
        send_all(sock, png, len) == 0) {
        char resp[256];
        ssize_t rn = recv(sock, resp, sizeof resp - 1, 0);
        if (rn > 0) {
            resp[rn] = 0;
            char *eol = strpbrk(resp, "\r\n");
            if (eol) *eol = 0;
            char *sp = strchr(resp, ' ');
            if (sp && (sp[1] == '2' || sp[1] == '3')) rc = 0;
        }
    }
    close(sock);
    return rc;
}

/* Extrae el package de la app en foreground parseando `dumpsys window`.
 * Formato buscado: "mCurrentFocus=Window{xxx u0 com.pkg/com.pkg.Activity}".
 * Deja out="" si no se pudo determinar. */
static void get_current_app(char *out, size_t outlen) {
    out[0] = '\0';
    int p[2];
    if (pipe(p) < 0) return;
    pid_t pid = fork();
    if (pid < 0) { close(p[0]); close(p[1]); return; }
    if (pid == 0) {
        close(p[0]);
        dup2(p[1], STDOUT_FILENO);
        int dn = open("/dev/null", O_WRONLY | O_CLOEXEC);
        if (dn >= 0) { dup2(dn, STDERR_FILENO); close(dn); }
        close(p[1]);
        execl("/system/bin/sh", "sh", "-c",
              "dumpsys window | grep -m1 mCurrentFocus", (char *)NULL);
        _exit(127);
    }
    close(p[1]);
    char buf[512];
    size_t total = 0;
    for (;;) {
        ssize_t r = read(p[0], buf + total, sizeof buf - 1 - total);
        if (r < 0) { if (errno == EINTR) continue; break; }
        if (r == 0) break;
        total += (size_t)r;
        if (total >= sizeof buf - 1) break;
    }
    close(p[0]);
    waitpid(pid, NULL, 0);
    if (total == 0) return;
    buf[total] = '\0';
    const char *u0 = strstr(buf, " u0 ");
    if (!u0) return;
    const char *pkg = u0 + 4;
    const char *end = pkg;
    while (*end && *end != '/' && *end != '}' && *end != ' ' &&
           *end != '\r' && *end != '\n') end++;
    size_t len = (size_t)(end - pkg);
    if (len == 0 || len >= outlen) return;
    memcpy(out, pkg, len);
    out[len] = '\0';
}

static int do_capture(const url_t *u, int tap_x, int tap_y) {
    size_t len = 0;
    char *png = capture_png(&len);
    if (!png) { fprintf(stderr, "screencap failed\n"); return -1; }

    char app[128];
    get_current_app(app, sizeof app);

    char path[1024];
    int pos = snprintf(path, sizeof path, "%s", u->path);
    if (pos < 0 || pos >= (int)sizeof path) { free(png); return -1; }
    char sep = strchr(u->path, '?') ? '&' : '?';
    if (tap_x >= 0 && tap_y >= 0) {
        int w = snprintf(path + pos, sizeof path - pos,
                         "%cx=%d&y=%d", sep, tap_x, tap_y);
        if (w > 0 && pos + w < (int)sizeof path) { pos += w; sep = '&'; }
    }
    if (app[0] && pos < (int)sizeof path - 1) {
        snprintf(path + pos, sizeof path - pos, "%capp=%s", sep, app);
    }

    int rc = post_png(u, path, png, len);
    free(png);
    return rc;
}

/* ---------- tap reader (parses getevent -l output) ---------- */

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void process_event_line(const char *line, tap_state_t *st, const url_t *u) {
    if (strstr(line, "ABS_MT_POSITION_X")) {
        const char *v = strrchr(line, ' ');
        if (v) { st->x = (int)strtol(v + 1, NULL, 16); st->has_x = 1; }
    } else if (strstr(line, "ABS_MT_POSITION_Y")) {
        const char *v = strrchr(line, ' ');
        if (v) { st->y = (int)strtol(v + 1, NULL, 16); st->has_y = 1; }
    } else if (strstr(line, "ABS_MT_TRACKING_ID")) {
        /* Protocolo B: tracking_id != -1 al bajar el dedo, = 0xffffffff al levantar.
         * En emulador ranchu no hay BTN_TOUCH, solo este evento. */
        const char *v = strrchr(line, ' ');
        if (v) {
            unsigned long id = strtoul(v + 1, NULL, 16);
            st->touch_down = (id != 0xffffffffUL);
        }
    } else if (strstr(line, "BTN_TOUCH")) {
        if (strstr(line, "DOWN")) {
            st->touch_down = 1;
        } else if (strstr(line, "UP")) {
            st->touch_down = 0;
        }
    } else if (strstr(line, "SYN_REPORT")) {
        if (st->touch_down && st->has_x && st->has_y) {
            printf("[tap] x=%d y=%d -> capture\n", st->x, st->y);
            do_capture(u, st->x, st->y);
            st->has_x = st->has_y = 0;
        }
    }
}

static int cmd_tap_stream(const url_t *u, int interval_ms) {
    int p[2];
    if (pipe(p) < 0) { perror("pipe"); return 1; }
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); return 1; }
    if (pid == 0) {
        close(p[0]);
        dup2(p[1], STDOUT_FILENO);
        int dn = open("/dev/null", O_WRONLY | O_CLOEXEC);
        if (dn >= 0) { dup2(dn, STDERR_FILENO); close(dn); }
        close(p[1]);
        execl("/system/bin/getevent", "getevent", "-l", (char *)NULL);
        _exit(127);
    }
    close(p[1]);
    int fd = p[0];
    fcntl(fd, F_SETFL, O_NONBLOCK);

    printf("tap-stream: periodicas cada %d ms + por tap, -> http://%s:%d%s\n",
           interval_ms, u->host, u->port, u->path);
    printf("(getevent en pid=%d, lee /dev/input/event*)\n", pid);

    char line_buf[1024];
    size_t line_pos = 0;
    tap_state_t st = {0};
    long last_periodic = now_ms();
    unsigned long n_periodic = 0;

    for (;;) {
        long elapsed = now_ms() - last_periodic;
        long remain = interval_ms - elapsed;
        if (remain < 0) remain = 0;

        struct timeval tv = { remain / 1000, (remain % 1000) * 1000 };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        int rv = select(fd + 1, &rfds, NULL, NULL, &tv);

        if (rv > 0 && FD_ISSET(fd, &rfds)) {
            char buf[4096];
            ssize_t n = read(fd, buf, sizeof buf);
            if (n > 0) {
                for (ssize_t i = 0; i < n; i++) {
                    char c = buf[i];
                    if (c == '\n' || line_pos == sizeof line_buf - 1) {
                        line_buf[line_pos] = 0;
                        /* getevent rellena con spaces hasta ancho fijo; recortar
                         * el trailing whitespace (y \r) para que strrchr encuentre
                         * el espacio justo antes del valor hex. */
                        while (line_pos > 0) {
                            char t = line_buf[line_pos - 1];
                            if (t != ' ' && t != '\t' && t != '\r') break;
                            line_buf[--line_pos] = 0;
                        }
                        process_event_line(line_buf, &st, u);
                        line_pos = 0;
                    } else {
                        line_buf[line_pos++] = c;
                    }
                }
            }
        }

        if (now_ms() - last_periodic >= interval_ms) {
            if (do_capture(u, -1, -1) == 0) {
                printf("[periodic %lu] OK\n", n_periodic++);
            }
            last_periodic = now_ms();
        }
    }
}

/* ---------- main ---------- */

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (argc < 3) {
        fprintf(stderr,
            "usage:\n"
            "  %s shot       URL                 una captura\n"
            "  %s stream     URL [interval_ms]   periodicas\n"
            "  %s tap-stream URL [interval_ms]   periodicas + por tap (red dot)\n"
            "URL: http://host[:port][/path]\n",
            argv[0], argv[0], argv[0]);
        return 1;
    }
    url_t u;
    if (parse_url(argv[2], &u) < 0) {
        fprintf(stderr, "URL invalida: %s\n", argv[2]);
        return 1;
    }
    if (strcmp(argv[1], "shot") == 0) {
        return do_capture(&u, -1, -1) == 0 ? 0 : 1;
    }
    if (strcmp(argv[1], "stream") == 0) {
        int ms = (argc >= 4) ? atoi(argv[3]) : 1000;
        if (ms < 50) ms = 50;
        printf("streaming a http://%s:%d%s cada %d ms\n", u.host, u.port, u.path, ms);
        unsigned long n = 0;
        for (;;) {
            if (do_capture(&u, -1, -1) == 0) printf("[%lu] OK\n", n++);
            struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
            nanosleep(&ts, NULL);
        }
    }
    if (strcmp(argv[1], "tap-stream") == 0) {
        int ms = (argc >= 4) ? atoi(argv[3]) : 1000;
        if (ms < 50) ms = 50;
        return cmd_tap_stream(&u, ms);
    }
    fprintf(stderr, "comando desconocido: %s\n", argv[1]);
    return 1;
}
