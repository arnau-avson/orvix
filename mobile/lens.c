/*
 * lens.c - native screenshot streamer
 *
 * Wrapper minimo en C alrededor de /system/bin/screencap. Captura el frame
 * en PNG y lo envia via HTTP POST al backend de Lens. Pensado para correr
 * en el emulador via `adb shell` (uid shell), no como app instalada.
 *
 * Build (NDK clang, x86_64 Android):
 *   x86_64-linux-android30-clang -O3 -o lens lens.c
 *
 * Use:
 *   lens shot   URL                     una sola captura
 *   lens stream URL [interval_ms]       bucle continuo (min 50 ms)
 *
 *   URL: http://host[:port][/path]      (sin HTTPS)
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

/* spawn /system/bin/screencap -p, slurp PNG bytes into a heap buffer */
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
        execlp("screencap", "screencap", "-p", (char *)NULL);
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

static int post_png(const url_t *u, const char *png, size_t len) {
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
        u->path, u->host, u->port, len);
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

static int do_one(const url_t *u) {
    size_t len = 0;
    char *png = capture_png(&len);
    if (!png) { fprintf(stderr, "screencap failed\n"); return -1; }
    int rc = post_png(u, png, len);
    free(png);
    return rc;
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (argc < 3) {
        fprintf(stderr,
            "usage:\n"
            "  %s shot   URL                una sola captura\n"
            "  %s stream URL [interval_ms]  bucle continuo (min 50 ms)\n"
            "URL: http://host[:port][/path]\n", argv[0], argv[0]);
        return 1;
    }
    url_t u;
    if (parse_url(argv[2], &u) < 0) {
        fprintf(stderr, "URL invalida: %s\n", argv[2]);
        return 1;
    }
    if (strcmp(argv[1], "shot") == 0) {
        int rc = do_one(&u);
        if (rc == 0) printf("OK -> http://%s:%d%s\n", u.host, u.port, u.path);
        return rc == 0 ? 0 : 1;
    }
    if (strcmp(argv[1], "stream") == 0) {
        int ms = (argc >= 4) ? atoi(argv[3]) : 1000;
        if (ms < 50) ms = 50;
        printf("streaming a http://%s:%d%s cada %d ms (Ctrl-C para parar)\n",
               u.host, u.port, u.path, ms);
        unsigned long n = 0;
        for (;;) {
            if (do_one(&u) == 0) printf("[%lu] OK\n", n++);
            struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
            nanosleep(&ts, NULL);
        }
    }
    fprintf(stderr, "comando desconocido: %s\n", argv[1]);
    return 1;
}